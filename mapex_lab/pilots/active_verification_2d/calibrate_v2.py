"""Offline error-model fit on V1 DEVELOPMENT snapshots only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

from .core import PolicyInput, UNKNOWN
from .predictor import file_hash
from .risk_model import feature_grid, fit_logistic, ErrorRiskModel
from .run import write_json, write_csv

BASE = Path(__file__).resolve().parent


def dataset(primary, protocol):
    xs, ys, layout_labels, weights, sources = [], [], [], [], []
    rng = np.random.default_rng(protocol["risk_sampling_seed"])
    for layout in protocol["development_layouts"]:
        if layout in protocol["confirmation_layouts"]:
            raise ValueError("Development/confirmation leakage")
        with np.load(BASE/"assets"/(layout+".npz")) as z:
            truth, domain = z["occupied"].copy(), z["domain"].copy()
        evaluation = domain | (truth & ndi.binary_dilation(domain, iterations=3))
        for warm in (5, 15):
            case = layout+"__warm"+str(warm)
            path = primary/"raw"/case/"initial.npz"
            with np.load(path) as z:
                s = PolicyInput(z["observed"].copy(), z["mean"].copy(), z["variance"].copy(),
                                z["predictions"].copy(), tuple(z["pose"]), .1, 8.)
            indices = np.flatnonzero(((s.observed == UNKNOWN) & evaluation).ravel())
            if len(indices) > protocol["risk_max_cells_per_state"]:
                indices = np.sort(rng.choice(indices, protocol["risk_max_cells_per_state"], replace=False))
            if not len(indices):
                raise ValueError("No development unknown cells: " + case)
            x = feature_grid(s).reshape(-1, 6)[indices]
            y = ((s.mean >= .5) != truth).ravel()[indices].astype(float)
            xs.append(x); ys.append(y)
            layout_labels.extend([layout]*len(indices))
            weights.extend([1/len(indices)]*len(indices))  # each warm state equal weight
            sources.append(dict(case=case, initial_sha256=file_hash(path), cells=len(y),
                                errors=int(y.sum()), asset_sha256=file_hash(BASE/"assets"/(layout+".npz"))))
    return np.concatenate(xs), np.concatenate(ys), np.asarray(weights), np.asarray(layout_labels), sources


def metrics(y, p, weights):
    weights = weights/weights.sum()
    q = np.clip(p, 1e-8, 1-1e-8)
    return dict(brier=float(np.sum(weights*(p-y)**2)),
                log_loss=float(-np.sum(weights*(y*np.log(q)+(1-y)*np.log(1-q)))),
                observed_error_rate=float(np.sum(weights*y)), estimated_error_rate=float(np.sum(weights*p)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v2")
    args = parser.parse_args()
    cfg = json.loads((BASE/"protocol_v2.json").read_text())
    x, y, weights, labels, sources = dataset(args.primary, cfg)
    folds = []
    for layout in cfg["development_layouts"]:
        train, test = labels != layout, labels == layout
        fitted = fit_logistic(x[train], y[train], weights[train], cfg["risk_l2"])
        p = ErrorRiskModel(fitted).predict_features(x[test])
        prior = np.full(test.sum(), fitted["prevalence"])
        a, b = metrics(y[test], p, weights[test]), metrics(y[test], prior, weights[test])
        folds.append(dict(layout=layout, cells=int(test.sum()), **a,
                          constant_prior_brier=b["brier"], delta_brier=a["brier"]-b["brier"]))
    fitted = fit_logistic(x, y, weights, cfg["risk_l2"])
    payload = dict(status="FROZEN_DEVELOPMENT_ONLY_ERROR_ESTIMATOR", model=fitted,
                   protocol_sha256=file_hash(BASE/"protocol_v2.json"),
                   training_code_sha256=file_hash(__file__), risk_code_sha256=file_hash(BASE/"risk_model.py"),
                   primary_provenance_sha256=file_hash(args.primary/"provenance.json"), sources=sources,
                   confirmation_layouts_excluded=cfg["confirmation_layouts"], leave_layout_out=folds,
                   fit_metrics=metrics(y, ErrorRiskModel(fitted).predict_features(x), weights),
                   caveat="Cross-layout folds are diagnostics on development. No tuning from these outcomes; no certified calibration, no confirmation labels used.")
    path = args.output/"error_model.json"
    if path.exists():
        raise RuntimeError("Do not overwrite a frozen error model; choose a new experiment")
    write_json(path, payload)
    write_csv(args.output/"risk_cross_layout.csv", folds)
    print("RISK_FIT", json.dumps(dict(cells=len(y), folds=folds)), flush=True)


if __name__ == "__main__":
    main()
