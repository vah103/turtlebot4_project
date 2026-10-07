"""Persistent real three-member LaMa inference and provenance-aware cache."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time

import numpy as np

from .core import array_hash, UNKNOWN


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(b)
    return digest.hexdigest()


class RealLamaPredictor:
    def __init__(self, repo_root, mapex_root, cache_dir, worker_python=None):
        self.repo_root = Path(repo_root)
        self.mapex_root = Path(mapex_root)
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        if worker_python:
            os.environ["MAPEX_LAMA_PYTHON"] = str(worker_python)
        script = self.repo_root / "mapex_lab/scripts/mapex_lama_bridge.py"
        spec = importlib.util.spec_from_file_location("pilot_lama_bridge", str(script))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.bridge = module.LamaEnsembleBridge(self.mapex_root, "cpu", {"transform_variant": "default_map_eval"})
        self.provenance = dict(
            type="REAL_LAMA_ENSEMBLE_NO_SYNTHETIC_PREDICTION",
            device="cpu", transform="default_map_eval",
            worker_sha256=file_hash(script.with_name("mapex_lama_worker.py")),
            bridge_sha256=file_hash(script),
            upstream_preprocessing_sha256=file_hash(self.mapex_root / "scripts/lama_pred_utils.py"),
            model_checkpoints=[dict(path=p, sha256=file_hash(p)) for p in self.bridge.source_names],
            model_configs=[dict(path=str(Path(p).parent.parent / "config.yaml"),
                                sha256=file_hash(Path(p).parent.parent / "config.yaml")) for p in self.bridge.source_names])
        self.identity = hashlib.sha256(json.dumps(self.provenance, sort_keys=True).encode()).hexdigest()
        self.calls = 0
        self.cache_hits = 0
        self.inference_s = 0.0

    def predict(self, observed):
        observed = np.asarray(observed, dtype=np.float32)
        key = hashlib.sha256((self.identity + array_hash(observed)).encode()).hexdigest()
        output = self.cache_dir / (key + ".npz")
        if output.exists():
            with np.load(output, allow_pickle=False) as z:
                if str(z["identity"]) != self.identity or str(z["observed_hash"]) != array_hash(observed):
                    raise RuntimeError("Inference cache provenance mismatch")
                result = tuple(z[k].copy() for k in ("predictions", "mean", "variance"))
            self.cache_hits += 1
            return result
        start = time.perf_counter()
        predictions, padded, pt, pl = self.bridge.predict_maps(observed)
        mean = self.bridge.compute_mean_map(predictions, padded)
        variance = self.bridge.compute_variance_map(predictions, padded)
        h, w = observed.shape
        predictions = predictions[:, pt:pt+h, pl:pl+w].copy()
        mean, variance = mean[pt:pt+h, pl:pl+w].copy(), variance[pt:pt+h, pl:pl+w].copy()
        if mean.shape != observed.shape or predictions.shape != (3,) + observed.shape:
            raise RuntimeError("Unexpected LaMa padding geometry")
        known = observed != UNKNOWN
        # The completion contract preserves actual observations for every member.
        predictions[:, known] = observed[known]
        mean[known] = observed[known]
        variance[known] = 0.0
        elapsed = time.perf_counter() - start
        self.inference_s += elapsed
        self.calls += 1
        temp = output.with_suffix(".tmp")
        with temp.open("wb") as stream:
            np.savez_compressed(stream, predictions=predictions, mean=mean, variance=variance,
                                observed_hash=array_hash(observed), identity=self.identity,
                                inference_s=elapsed)
        temp.replace(output)
        return predictions, mean, variance

    def close(self):
        self.bridge.close()
