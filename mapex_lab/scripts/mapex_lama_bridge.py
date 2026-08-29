#!/usr/bin/env python3
"""ROS-side bridge to the persistent legacy MapEx LaMa worker."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np


class LamaEnsembleBridge:
    """Drop-in replacement for mapex.LamaEnsemble using a separate Python env."""

    def __init__(self, mapex_root: Path, device: str, prediction_config: dict) -> None:
        self.mapex_root = mapex_root.expanduser().resolve()
        self.device = device
        self.prediction_config = prediction_config
        self._tmp = tempfile.TemporaryDirectory(prefix="mapex_lama_")
        self._counter = 0

        default_python = (
            Path.home() / "miniforge3" / "envs" / "lama" / "bin" / "python"
        )
        worker_python = Path(
            os.environ.get("MAPEX_LAMA_PYTHON", str(default_python))
        ).expanduser()
        if not worker_python.is_file():
            raise RuntimeError(
                "MapEx LaMa worker Python not found: "
                f"{worker_python}. Create the official 'lama' environment or set "
                "MAPEX_LAMA_PYTHON to its python executable."
            )

        worker_script = Path(__file__).with_name("mapex_lama_worker.py").resolve()
        command = [
            str(worker_python),
            str(worker_script),
            "--mapex-root",
            str(self.mapex_root),
            "--device",
            self.device,
            "--transform-variant",
            str(prediction_config["transform_variant"]),
        ]
        for key, flag in (
            ("checkpoint_g1", "--checkpoint-g1"),
            ("checkpoint_g2", "--checkpoint-g2"),
            ("checkpoint_g3", "--checkpoint-g3"),
        ):
            value = prediction_config.get(key)
            if value is not None:
                command.extend([flag, str(value)])

        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )
        ready = self._read_response()
        if ready.get("status") != "ready":
            self.close()
            raise RuntimeError(
                "MapEx LaMa worker failed during startup: "
                f"{ready.get('error', ready)}"
            )
        self.source_names = list(ready.get("sources", []))

    def _read_response(self) -> dict:
        if self.process.stdout is None:
            raise RuntimeError("MapEx LaMa worker stdout is unavailable")
        line = self.process.stdout.readline()
        if not line:
            return {
                "status": "fatal",
                "error": f"worker exited with code {self.process.poll()}",
            }
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Invalid response from MapEx LaMa worker: {line.rstrip()}"
            ) from exc
        return payload

    def predict_mean_variance(
        self,
        observed_map: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
        if self.process.poll() is not None:
            raise RuntimeError(
                f"MapEx LaMa worker already exited with code {self.process.returncode}"
            )
        if self.process.stdin is None:
            raise RuntimeError("MapEx LaMa worker stdin is unavailable")

        self._counter += 1
        root = Path(self._tmp.name)
        input_path = root / f"input_{self._counter:06d}.npy"
        output_path = root / f"output_{self._counter:06d}.npz"
        np.save(input_path, observed_map.astype(np.float32, copy=False))

        request = {
            "cmd": "predict",
            "input": str(input_path),
            "output": str(output_path),
        }
        self.process.stdin.write(json.dumps(request, separators=(",", ":")) + "\n")
        self.process.stdin.flush()

        response = self._read_response()
        if response.get("status") != "ok":
            raise RuntimeError(
                "MapEx LaMa worker prediction failed: "
                f"{response.get('error', response)}"
            )
        if not output_path.is_file():
            raise RuntimeError("MapEx LaMa worker returned success without output")

        with np.load(output_path) as data:
            mean_map = data["mean_map"].astype(np.float32, copy=True)
            variance_map = data["variance_map"].astype(np.float32, copy=True)
            padded_observed = data["padded_observed"].astype(np.float32, copy=True)
            pad_top = int(data["pad_top"])
            pad_left = int(data["pad_left"])

        input_path.unlink(missing_ok=True)
        output_path.unlink(missing_ok=True)
        return mean_map, variance_map, padded_observed, pad_top, pad_left

    def close(self) -> None:
        process = getattr(self, "process", None)
        if process is not None and process.poll() is None:
            try:
                if process.stdin is not None:
                    process.stdin.write('{"cmd":"shutdown"}\n')
                    process.stdin.flush()
            except Exception:
                pass
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
        tmp = getattr(self, "_tmp", None)
        if tmp is not None:
            tmp.cleanup()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
