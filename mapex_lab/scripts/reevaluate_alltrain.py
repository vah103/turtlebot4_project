#!/usr/bin/env python3
"""Regenerate paper-style all-training predictions and IoU/TU for saved runs.

This is an offline-only helper. It loads the shared MapEx whole-training LaMa
checkpoint once, generates alltrain predictions for each supplied run, and then
invokes the matching evaluator:

- Nearest Frontier: evaluate_nf_profiled.py
- MapEx: evaluate_mapex_run.py

The recorded exploration data are never changed. Generated predictions live in
``evaluation/alltrain`` and the usual ``evaluation.csv`` / ``evaluation.json``
are refreshed by the evaluator.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
from pathlib import Path
import sys

import numpy as np

import predict_alltrain_offline as predictor
import evaluate_mapex_run
import evaluate_nf_profiled


def _load_predictor(mapex_root: Path, checkpoint_arg: str, device: str):
    checkpoint = Path(checkpoint_arg).expanduser()
    if not checkpoint.is_absolute():
        checkpoint = mapex_root / checkpoint
    if checkpoint.is_dir():
        checkpoint = checkpoint / "models" / "best.ckpt"
    if not checkpoint.is_file() or checkpoint.parent.name != "models":
        raise FileNotFoundError(
            "Supply a MapEx all-training checkpoint under <model>/models/: "
            f"{checkpoint}"
        )

    model_dir = checkpoint.parent.parent
    if not (model_dir / "config.yaml").is_file():
        raise FileNotFoundError(f"Missing model config.yaml under {model_dir}")

    sys.path.insert(0, str(mapex_root / "lama"))
    sys.path.insert(0, str(mapex_root / "scripts"))
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from lama_pred_utils import (
            load_lama_model,
            get_lama_transform,
            convert_obsimg_to_model_input,
        )

        model = load_lama_model(
            str(model_dir), checkpoint_name=checkpoint.name, device=device
        )
        model.eval()
        transform = get_lama_transform("default_map_eval", (512, 512))

    def predict(observed):
        with contextlib.redirect_stdout(sys.stderr), torch.no_grad():
            batch, _ = convert_obsimg_to_model_input(
                np.stack([observed] * 3, axis=2), transform, device
            )
            return model(batch)["inpainted"][0, 0].detach().float().cpu().numpy()

    return checkpoint, predict


def _method(run_dir: Path) -> str:
    if (run_dir / "policy_decisions.csv").is_file():
        return "nearest"
    if (run_dir / "decisions.csv").is_file():
        return "mapex"
    raise FileNotFoundError(
        f"{run_dir} has neither policy_decisions.csv nor decisions.csv"
    )


def _evaluate(run_dir: Path, method: str, ground_truth: Path, roi: Path):
    if method == "nearest":
        return evaluate_nf_profiled.evaluate_run(run_dir, ground_truth, roi)
    return evaluate_mapex_run.evaluate_run(run_dir, ground_truth, roi)


def _verify_reused_manifest(manifest_path: Path, checkpoint: Path) -> None:
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("prediction_source") != "alltrain":
        raise ValueError(f"{manifest_path} is not an alltrain manifest")
    checkpoint_meta = payload.get("checkpoint") or {}
    actual_sha = checkpoint_meta.get("sha256")
    expected_sha = predictor.sha256(checkpoint)
    if actual_sha != expected_sha:
        raise ValueError(
            f"{manifest_path} was generated with a different checkpoint; "
            "rerun with --overwrite-alltrain"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "run_dirs",
        nargs="+",
        help="saved NF/MapEx run directories; shell globs may be used",
    )
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--roi", required=True)
    parser.add_argument(
        "--mapex-root",
        default=os.environ.get("MAPEX_ROOT", str(Path.home() / "MapEx")),
    )
    parser.add_argument(
        "--checkpoint",
        default="pretrained_models/weights/big_lama",
    )
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument(
        "--overwrite-alltrain",
        action="store_true",
        help="replace an existing evaluation/alltrain directory for each run",
    )
    parser.add_argument(
        "--evaluate-only",
        action="store_true",
        help="skip LaMa inference and only evaluate existing valid alltrain manifests",
    )
    args = parser.parse_args()

    ground_truth = Path(args.ground_truth).expanduser().resolve()
    roi = Path(args.roi).expanduser().resolve()
    if not ground_truth.is_file():
        parser.error(f"ground truth not found: {ground_truth}")
    if not roi.is_file():
        parser.error(f"ROI not found: {roi}")

    run_dirs = [Path(item).expanduser().resolve() for item in args.run_dirs]
    for run_dir in run_dirs:
        if not run_dir.is_dir():
            parser.error(f"run directory not found: {run_dir}")

    checkpoint = None
    predict = None
    if not args.evaluate_only:
        mapex_root = Path(args.mapex_root).expanduser().resolve()
        checkpoint, predict = _load_predictor(
            mapex_root, args.checkpoint, args.device
        )

    results = []
    for run_dir in run_dirs:
        method = _method(run_dir)
        print(f"=== {run_dir.name} ({method}) ===", flush=True)

        manifest = run_dir / "evaluation" / "alltrain" / "manifest.json"
        if not args.evaluate_only:
            try:
                manifest = predictor.generate(
                    run_dir,
                    checkpoint,
                    predict,
                    overwrite=args.overwrite_alltrain,
                )
                print(f"alltrain manifest: {manifest}", flush=True)
            except FileExistsError:
                if not manifest.is_file():
                    raise
                _verify_reused_manifest(manifest, checkpoint)
                print(
                    f"alltrain already exists with the requested checkpoint: {manifest}; "
                    "reusing it (pass --overwrite-alltrain to regenerate)",
                    flush=True,
                )

        result = _evaluate(run_dir, method, ground_truth, roi)
        status = result.get("status", "unknown")
        source = result.get("prediction_source")
        if status != "ok":
            raise RuntimeError(
                f"{run_dir.name}: evaluation failed: {status}: "
                f"{result.get('reason', 'no reason')}"
            )
        if source != "alltrain":
            raise RuntimeError(
                f"{run_dir.name}: expected prediction_source=alltrain, got {source}"
            )

        row = {
            "run": run_dir.name,
            "method": method,
            "prediction_source": source,
            "final_occupied_iou": result["final_occupied_iou"],
            "final_tu": result["final_tu"],
            "evaluated_decisions": result["evaluated_decisions"],
        }
        results.append(row)
        print(json.dumps(row, indent=2), flush=True)

    print("=== ALLTRAIN RE-EVALUATION COMPLETE ===")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
