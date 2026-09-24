#!/usr/bin/env python3
"""R004 H044 frozen V3 free-error explanatory diagnostic."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import distance_transform_edt

from mapex_lab.analysis.r004 import evaluate_prediction_vs_final_observed as base

BANDS = ("boundary", "near_interior", "deep_occupied")
BINS = ("[0.00,0.25)", "[0.25,0.50)", "[0.50,0.75)", "[0.75,1.00]")
PRIMARY = (
    "false_free_fraction",
    "supported_future_free_miss_fraction",
    "false_free_union_share",
    "missed_free_union_share",
    "free_support_coverage",
    "unsupported_future_free_fraction",
    "deep_false_free_fraction",
    "free_precision",
    "free_iou",
    "free_precision_excluding_deep",
    "free_iou_excluding_deep",
    "free_precision_deep_delta",
    "free_iou_deep_delta",
)


def classify(prediction, final_truth):
    pred_occupied = np.asarray(prediction, dtype=float) > 0.5
    truth_occupied = np.asarray(final_truth, dtype=bool)
    tp_free = (~pred_occupied) & (~truth_occupied)
    false_free = (~pred_occupied) & truth_occupied
    missed_free = pred_occupied & (~truth_occupied)
    return tp_free, false_free, missed_free


def occupied_depth(final):
    final = np.asarray(final)
    if not np.any(final == 0):
        raise ValueError("FinalObserved contains no known-free seed for Euclidean depth")
    return distance_transform_edt(final != 0)


def depth_band(depth):
    depth = np.asarray(depth, dtype=float)
    out = np.full(depth.shape, "", dtype="U14")
    out[depth <= 1.0] = "boundary"
    out[(depth > 1.0) & (depth <= 2.0)] = "near_interior"
    out[depth > 2.0] = "deep_occupied"
    return out


def decision_metrics(pred, truth, full_f_free, e_free, unsupported_free, depth):
    tp, ff, miss = classify(pred, truth)
    union = int(np.sum(tp | ff | miss))
    ntp, nff, nmiss = int(tp.sum()), int(ff.sum()), int(miss.sum())
    predicted_free = ntp + nff
    supported_free = ntp + nmiss
    deep = np.asarray(depth) > 2.0
    nff_deep = int(np.sum(ff & deep))
    nff_kept = nff - nff_deep
    out = {
        "tp_free_count": ntp,
        "false_free_count": nff,
        "missed_free_count": nmiss,
        "free_iou_union_count": union,
        "false_free_fraction": base.div(nff, predicted_free),
        "supported_future_free_miss_fraction": base.div(nmiss, supported_free),
        "false_free_union_share": base.div(nff, union),
        "missed_free_union_share": base.div(nmiss, union),
        "free_support_coverage": base.div(e_free, full_f_free),
        "supported_future_free_count": e_free,
        "unsupported_future_free_count": unsupported_free,
        "unsupported_future_free_fraction": base.div(unsupported_free, full_f_free),
        "free_precision": base.div(ntp, ntp + nff),
        "free_recall": base.div(ntp, ntp + nmiss),
        "free_iou": base.div(ntp, union),
        "free_precision_excluding_deep": base.div(ntp, ntp + nff_kept),
        "free_iou_excluding_deep": base.div(ntp, union - nff_deep),
        "deep_false_free_fraction": base.div(nff_deep, nff),
    }
    out["free_precision_deep_delta"] = out["free_precision_excluding_deep"] - out["free_precision"]
    out["free_iou_deep_delta"] = out["free_iou_excluding_deep"] - out["free_iou"]
    bands = depth_band(depth)
    for band in BANDS:
        denom = int(np.sum(truth & (bands == band)))
        count = int(np.sum(ff & (bands == band)))
        out[f"scoreable_final_occupied_{band}_count"] = denom
        out[f"false_free_{band}_count"] = count
        out[f"false_free_{band}_rate"] = base.div(count, denom)
        out[f"false_free_{band}_share"] = base.div(count, nff)
    return out


def analyze_run(run):
    final, final_file, fallback = base.final_snapshot(run)
    if not np.any(final > 0):
        raise ValueError(f"{run.name}: FinalObserved has no occupied evaluation cells")
    depth = occupied_depth(final)
    table = base.read_csv(run / "decisions.csv")
    rows = []
    overlay_inputs = []
    for k, decision in enumerate(table, 1):
        obs, prediction, support = base.prediction_canvas(run, decision, final.shape)
        F = (obs < 0) & (final >= 0)
        E = F & support
        final_free = final == 0
        e_free = int(np.sum(E & final_free))
        f_free = int(np.sum(F & final_free))
        selected_pred = prediction[E]
        selected_truth = final[E] > 0
        selected_depth = depth[E]
        row = {
            "run_id": run.name,
            "decision_id": int(decision["decision_id"]),
            "decision_index": k,
            "decision_count": len(table),
            "decision_progress": base.progress(k, len(table)),
            "progress_bin": base.pbin(base.progress(k, len(table))),
            "final_snapshot": final_file,
            "fallback_final_snapshot": int(fallback),
            "scoreable_count": int(E.sum()),
        }
        row.update(decision_metrics(selected_pred, selected_truth, f_free, e_free, f_free - e_free, selected_depth))
        rows.append(row)
        overlay_inputs.append((decision, obs, prediction, support, F, E, final, depth))
    return rows, overlay_inputs


def metric_names():
    depth_metrics = []
    for band in BANDS:
        depth_metrics.extend((f"false_free_{band}_rate", f"false_free_{band}_share"))
    return PRIMARY + tuple(depth_metrics)


def aggregate(rows):
    per_run = []
    for rid in base.RUNS:
        for progress_bin in BINS:
            selected = [r for r in rows if r["run_id"] == rid and r["progress_bin"] == progress_bin]
            record = {"aggregation": "run_median", "run_id": rid, "progress_bin": progress_bin,
                      "decision_count": len(selected)}
            for metric in metric_names():
                values = base.finite([r[metric] for r in selected])
                record[metric] = float(np.median(values)) if values.size else math.nan
            for name in ("supported_future_free_count", "unsupported_future_free_count"):
                record[name] = int(sum(r[name] for r in selected))
            per_run.append(record)
    macro = []
    for progress_bin in BINS:
        selected = [r for r in per_run if r["progress_bin"] == progress_bin]
        record = {"progress_bin": progress_bin,
                  "decision_count": int(sum(r["decision_count"] for r in selected))}
        for metric in metric_names():
            stats = base.summarize([r[metric] for r in selected])
            record.update({f"{metric}_{key}": value for key, value in stats.items()})
        for name in ("supported_future_free_count", "unsupported_future_free_count"):
            record[name] = int(sum(r[name] for r in selected))
        macro.append(record)
    return per_run, macro


def plot_metric(macro, output, filename, series, title, ylabel):
    x = np.arange(len(macro))
    fig, ax = plt.subplots(figsize=(8, 5))
    for metric, label, style in series:
        ax.errorbar(x, [r[f"{metric}_mean"] for r in macro],
                    yerr=[r[f"{metric}_std"] for r in macro], fmt=style,
                    capsize=3, label=label)
    ax.set_xticks(x, BINS, rotation=15)
    ax.set(xlabel="Normalized decision-progress bin", ylabel=ylabel, title=title)
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / filename, dpi=160)
    plt.close(fig)


def figures(macro, output):
    output.mkdir(parents=True, exist_ok=True)
    plot_metric(macro, output, "false_free_vs_missed_free_union_share.png",
                (("false_free_union_share", "FalseFree union share", "o-"),
                 ("missed_free_union_share", "MissedFree union share", "s--")),
                "Free-IoU union error decomposition", "Run-macro share mean ± std")
    plot_metric(macro, output, "false_free_fraction_vs_supported_future_free_miss_fraction.png",
                (("false_free_fraction", "FalseFree / predicted free", "o-"),
                 ("supported_future_free_miss_fraction", "SupportedFutureFreeMissFraction", "s--")),
                "Free error fractions", "Run-macro fraction mean ± std")
    plot_metric(macro, output, "false_free_depth_band_distribution.png",
                tuple((f"false_free_{b}_share", b.replace("_", " "), style)
                      for b, style in zip(BANDS, ("o-", "s--", "^:"))),
                "FalseFree distribution across occupied-depth bands", "Run-macro FalseFree share mean ± std")
    plot_metric(macro, output, "false_free_rate_by_depth_band.png",
                tuple((f"false_free_{b}_rate", b.replace("_", " "), style)
                      for b, style in zip(BANDS, ("o-", "s--", "^:"))),
                "Denominator-controlled FalseFree rate", "Run-macro rate mean ± std")
    plot_metric(macro, output, "supported_future_free_miss_and_support.png",
                (("supported_future_free_miss_fraction", "SupportedFutureFreeMissFraction", "o-"),
                 ("free_support_coverage", "Free prediction-support coverage", "s--"),
                 ("unsupported_future_free_fraction", "Unsupported future-free fraction", "^:")),
                "Support-conditional future-free evidence", "Run-macro fraction mean ± std")
    plot_metric(macro, output, "deep_occupied_exclusion_sensitivity.png",
                (("free_precision", "Original free precision", "o-"),
                 ("free_precision_excluding_deep", "Precision excluding deep occupied", "o--"),
                 ("free_iou", "Original free IoU", "s-"),
                 ("free_iou_excluding_deep", "IoU excluding deep occupied", "s--")),
                "Deep-occupied exclusion sensitivity", "Run-macro metric mean ± std")


def overlays(run_id, inputs, output):
    records = []
    n = len(inputs)
    for position, k in (("early", 1), ("middle", 1 + round((n - 1) * 0.5)), ("late", n)):
        decision, obs, prediction, support, F, E, final, depth = inputs[k - 1]
        pred_occ = prediction > 0.5
        truth_occ = final > 0
        tp = E & (~pred_occ) & (~truth_occ)
        ff = E & (~pred_occ) & truth_occ
        miss = E & pred_occ & (~truth_occ)
        image = np.ones((*final.shape, 3), dtype=np.float32)
        image[obs >= 0] = (0.75, 0.75, 0.75)
        image[F & ~support] = (0.20, 0.45, 0.95)
        image[tp] = (0.20, 0.75, 0.25)
        image[miss] = (0.95, 0.55, 0.05)
        image[ff & (depth <= 1)] = (0.95, 0.15, 0.20)
        image[ff & (depth > 1) & (depth <= 2)] = (0.75, 0.10, 0.70)
        image[ff & (depth > 2)] = (0.25, 0.05, 0.35)
        visible = F | (obs >= 0)
        ys, xs = np.nonzero(visible)
        if ys.size:
            margin = 20
            r0, r1 = max(0, int(ys.min()) - margin), min(image.shape[0], int(ys.max()) + margin + 1)
            c0, c1 = max(0, int(xs.min()) - margin), min(image.shape[1], int(xs.max()) + margin + 1)
            image = image[r0:r1, c0:c1]
        path = output / f"{run_id}_{position}_decision_{int(decision['decision_id']):03d}.png"
        plt.imsave(path, image)
        records.append({"run_id": run_id, "position": position,
                        "decision_id": int(decision["decision_id"]),
                        "decision_progress": base.progress(k, n),
                        "file": str(path.name)})
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    all_rows, exclusions, overlay_manifest = [], [], []
    overlay_dir = args.output / "overlays"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    for rid in base.RUNS:
        run = args.data_root / "mapex_lab/experiments/mapex" / rid
        try:
            rows, inputs = analyze_run(run)
            all_rows.extend(rows)
            overlay_manifest.extend(overlays(rid, inputs, overlay_dir))
        except Exception as exc:
            exclusions.append({"run_id": rid, "reason": str(exc)})
    per_run, macro = aggregate(all_rows)
    args.output.mkdir(parents=True, exist_ok=True)
    base.write_csv(args.output / "free_error_decomposition_decisions.csv", all_rows)
    base.write_csv(args.output / "free_error_decomposition_run_bins.csv", per_run)
    base.write_csv(args.output / "free_error_overlay_manifest.csv", overlay_manifest)
    figures(macro, args.output / "figures")
    summary = {
        "schema": "r004_free_error_diagnostic_v3",
        "accepted_base_sha": "550fa35a082041972133700f9688130bc73fd6f3",
        "included_runs": sorted({r["run_id"] for r in all_rows}),
        "excluded_runs": exclusions,
        "decisions": len(all_rows),
        "counts": {name: int(sum(r[name] for r in all_rows)) for name in
                   ("tp_free_count", "false_free_count", "missed_free_count",
                    "supported_future_free_count", "unsupported_future_free_count")},
        "run_macro_progress_bins": macro,
        "interpretation_cases": {
            "A": "FalseFree/deep-occupied dominant; hypothesis strongly supported",
            "B": "deep FalseFree material and MissedFree also rises; partly supported",
            "C": "no disproportionate deep FalseFree or MissedFree dominant; not supported",
            "D": "geometry/occupancy semantics insufficient; inconclusive",
        },
        "scientific_case_selected": None,
    }
    (args.output / "free_error_decomposition_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    provenance = {
        "schema": "r004_free_error_provenance_v1",
        "method": "accepted R004 V3+V4 plus accepted free-error diagnostic V3",
        "accepted_base_sha": "550fa35a082041972133700f9688130bc73fd6f3",
        "threshold": "occupied iff p > 0.5",
        "scoreable_domain": "E = UnknownAtDecision ∩ KnownInFinalObserved ∩ PredictionSupportAtDecision",
        "final_observed": "accepted V3 canonical final/fallback validation",
        "prediction_support": "accepted V4 prediction-artifact geometry",
        "occupied_depth": "Euclidean center-to-center distance to nearest FinalObserved-known free cell; scipy.ndimage.distance_transform_edt; unknown is neither seed nor evaluation cell",
        "depth_bands_cells": {"boundary": "<=1", "near_interior": ">1 and <=2", "deep_occupied": ">2"},
        "aggregation": "per decision -> within-run/progress-bin median -> equal-run mean/sample std/median/count",
        "data_root": str(args.data_root.resolve()),
        "runs": list(base.RUNS),
        "exclusions": exclusions,
        "overlay_legend": {"gray": "already observed", "blue": "unsupported future-observed",
                           "green": "TP_free", "orange": "MissedFree", "red": "boundary FalseFree",
                           "magenta": "near-interior FalseFree", "dark purple": "deep-occupied FalseFree"},
    }
    (args.output / "free_error_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(json.dumps({"included_runs": len(summary["included_runs"]), "excluded_runs": exclusions,
                      "decisions": len(all_rows), "counts": summary["counts"]}, indent=2))


if __name__ == "__main__":
    main()
