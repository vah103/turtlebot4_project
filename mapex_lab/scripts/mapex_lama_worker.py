#!/usr/bin/env python3
"""Persistent LaMa inference worker for MapEx.

This process is intentionally ROS-free.  It runs inside the legacy MapEx/LaMa
Python environment while ``mapex_ros.py`` keeps ROS 2 Jazzy in system Python.
Communication uses JSON lines plus temporary NumPy files so the three ensemble
models are loaded only once.
"""

import argparse
import contextlib
import json
import os
import sys
import traceback

import numpy as np


def emit(payload):
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def resolve_models(mapex_root, checkpoint_values):
    if any(value is not None for value in checkpoint_values):
        if not all(value is not None for value in checkpoint_values):
            raise RuntimeError("Set all checkpoint_g1/g2/g3 values or leave all null")
        specs = []
        for value in checkpoint_values:
            path = os.path.expanduser(value)
            if not os.path.isabs(path):
                path = os.path.join(mapex_root, path)
            path = os.path.realpath(path)
            if os.path.isdir(path):
                model_dir = path
                checkpoint_name = "best.ckpt"
            elif os.path.isfile(path):
                if os.path.basename(os.path.dirname(path)) != "models":
                    raise RuntimeError(
                        "Configured checkpoint must live in <model>/models/: {}".format(path)
                    )
                model_dir = os.path.dirname(os.path.dirname(path))
                checkpoint_name = os.path.basename(path)
            else:
                raise RuntimeError("Configured checkpoint not found: {}".format(path))
            specs.append((model_dir, checkpoint_name))
        return specs

    ensemble_dir = os.environ.get(
        "MAPEX_ENSEMBLE_DIR",
        os.path.join(mapex_root, "pretrained_models", "weights", "lama_ensemble"),
    )
    ensemble_dir = os.path.realpath(os.path.expanduser(ensemble_dir))
    if not os.path.isdir(ensemble_dir):
        raise RuntimeError("MapEx ensemble directory not found: {}".format(ensemble_dir))

    member_dirs = sorted(
        os.path.join(ensemble_dir, name)
        for name in os.listdir(ensemble_dir)
        if os.path.isdir(os.path.join(ensemble_dir, name))
    )
    if len(member_dirs) != 3:
        raise RuntimeError(
            "Expected exactly 3 ensemble directories under {}; found {}".format(
                ensemble_dir, len(member_dirs)
            )
        )
    return [(path, "best.ckpt") for path in member_dirs]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mapex-root", required=True)
    parser.add_argument("--device", required=True)
    parser.add_argument("--transform-variant", default="default_map_eval")
    parser.add_argument("--checkpoint-g1", default=None)
    parser.add_argument("--checkpoint-g2", default=None)
    parser.add_argument("--checkpoint-g3", default=None)
    args = parser.parse_args()

    mapex_root = os.path.realpath(os.path.expanduser(args.mapex_root))
    lama_root = os.path.join(mapex_root, "lama")
    scripts_root = os.path.join(mapex_root, "scripts")
    if not os.path.isdir(lama_root):
        raise RuntimeError("MapEx LaMa submodule not found: {}".format(lama_root))
    if not os.path.isdir(scripts_root):
        raise RuntimeError("MapEx scripts directory not found: {}".format(scripts_root))

    sys.path.insert(0, lama_root)
    sys.path.insert(0, scripts_root)
    os.chdir(scripts_root)

    # Keep the JSON protocol on stdout clean; upstream MapEx/LaMa may print.
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from lama_pred_utils import (
            convert_obsimg_to_model_input,
            get_lama_transform,
            load_lama_model,
        )

        if args.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                "Requested device '{}', but CUDA is unavailable in the LaMa environment".format(
                    args.device
                )
            )

        map_transform = get_lama_transform(args.transform_variant, (512, 512))
        model_specs = resolve_models(
            mapex_root,
            [args.checkpoint_g1, args.checkpoint_g2, args.checkpoint_g3],
        )
        models = []
        source_names = []
        for model_dir, checkpoint_name in model_specs:
            model = load_lama_model(
                model_dir,
                checkpoint_name=checkpoint_name,
                device=args.device,
            )
            model.eval()
            models.append(model)
            source_names.append(os.path.join(model_dir, "models", checkpoint_name))

    emit({"status": "ready", "sources": source_names})

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
            command = request.get("cmd")
            if command == "shutdown":
                emit({"status": "bye"})
                return
            if command != "predict":
                raise RuntimeError("Unknown worker command: {}".format(command))

            observed_map = np.load(request["input"]).astype(np.float32, copy=False)
            observed_3channel = np.stack(
                [observed_map, observed_map, observed_map], axis=2
            )

            with contextlib.redirect_stdout(sys.stderr):
                input_batch, _mask = convert_obsimg_to_model_input(
                    observed_3channel,
                    map_transform,
                    args.device,
                )
                padded_observed = (
                    input_batch["image"][0, 0]
                    .detach()
                    .float()
                    .cpu()
                    .numpy()
                    .astype(np.float32, copy=False)
                )

                predictions = []
                with torch.no_grad():
                    for model in models:
                        batch = {}
                        for key, value in input_batch.items():
                            batch[key] = value.clone() if torch.is_tensor(value) else value
                        output = model(batch)
                        if not isinstance(output, dict) or "inpainted" not in output:
                            raise RuntimeError(
                                "Official LaMa model output does not contain 'inpainted'"
                            )
                        predictions.append(output["inpainted"][0, 0].detach())

                prediction_stack = torch.stack(predictions, dim=0)
                mean_map = torch.mean(prediction_stack, dim=0).float().cpu().numpy()
                variance_map = torch.var(prediction_stack, dim=0).float().cpu().numpy()

            mean_map = np.nan_to_num(mean_map, nan=0.5, posinf=1.0, neginf=0.0)
            variance_map = np.nan_to_num(
                variance_map, nan=0.0, posinf=0.0, neginf=0.0
            )
            mean_map = np.clip(mean_map, 0.0, 1.0).astype(np.float32, copy=False)
            variance_map = np.maximum(variance_map, 0.0).astype(
                np.float32, copy=False
            )

            known = ~np.isclose(padded_observed, 0.5)
            mean_map[known] = padded_observed[known]
            variance_map[known] = 0.0

            input_h, input_w = observed_map.shape
            output_h, output_w = padded_observed.shape
            if output_h < input_h or output_w < input_w:
                raise RuntimeError("default_map_eval unexpectedly shrank the map")
            pad_top = (output_h - input_h) // 2
            pad_left = (output_w - input_w) // 2

            np.savez_compressed(
                request["output"],
                mean_map=mean_map,
                variance_map=variance_map,
                padded_observed=padded_observed,
                pad_top=np.int64(pad_top),
                pad_left=np.int64(pad_left),
            )
            emit({"status": "ok"})
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            emit({"status": "error", "error": str(exc)})


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        emit({"status": "fatal", "error": str(exc)})
        sys.exit(1)
