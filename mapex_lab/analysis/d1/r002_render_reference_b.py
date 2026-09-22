#!/usr/bin/env python3
"""Visualization-only Reference-B audit for frozen R002 / mpx_001.

This renderer does not redefine or modify R002 semantics. It imports the
accepted Reference-B helpers/constants from r002_gt_semantics.py and renders
all 35 mpx_001 decisions for human audit.

Per decision:
- B_GT_t is the frozen GT surface target inside U_t.
- B_PRED_t is the frozen predicted occupied boundary inside U_t.
- left panel: literal layered view (GT below, prediction above with alpha).
- right panel: accepted 0.10 m one-to-one matching diagnostic
  (matched endpoints green, unmatched GT black, unmatched prediction red).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

import r002_gt_semantics as r002


EXPECTED_DECISIONS = 35
OUTPUT_SCHEMA = "r002_reference_b_visual_audit_v1"


def _crop_bounds(
    support: np.ndarray,
    padding: int = 4,
) -> tuple[int, int, int, int]:
    if np.any(support):
        rows, cols = np.nonzero(support)
        r0 = max(0, int(rows.min()) - padding)
        r1 = min(support.shape[0], int(rows.max()) + padding + 1)
        c0 = max(0, int(cols.min()) - padding)
        c1 = min(support.shape[1], int(cols.max()) + padding + 1)
        return r0, r1, c0, c1
    return 0, support.shape[0], 0, support.shape[1]


def _matched_endpoint_masks(
    shape: tuple[int, int],
    pairs: list[dict],
) -> tuple[np.ndarray, np.ndarray]:
    matched_gt = np.zeros(shape, dtype=bool)
    matched_pred = np.zeros(shape, dtype=bool)
    for pair in pairs:
        gr = int(pair["gt_row"])
        gc = int(pair["gt_col"])
        pr = int(pair["pred_row"])
        pc = int(pair["pred_col"])
        matched_gt[gr, gc] = True
        matched_pred[pr, pc] = True
    return matched_gt, matched_pred


def _render_decision(
    output_path: Path,
    decision_id: int,
    projection: dict,
    b_gt: np.ndarray,
    b_pred: np.ndarray,
    matching_010: dict,
) -> dict:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    r0, r1, c0, c1 = _crop_bounds(projection["support"])

    bgt = b_gt[r0:r1, c0:c1]
    bpr = b_pred[r0:r1, c0:c1]

    matched_gt_full, matched_pred_full = _matched_endpoint_masks(
        b_gt.shape,
        matching_010["pairs"],
    )
    matched_gt = matched_gt_full[r0:r1, c0:c1]
    matched_pred = matched_pred_full[r0:r1, c0:c1]

    # Literal layered view: B_GT below, B_PRED above.
    layered_base = np.ones(bgt.shape + (3,), dtype=np.float32)
    layered_base[bgt] = np.asarray([0.12, 0.12, 0.12])

    pred_overlay = np.zeros(bgt.shape + (4,), dtype=np.float32)
    pred_overlay[bpr] = np.asarray([0.90, 0.10, 0.10, 0.55])

    # Official 0.10 m one-to-one matching diagnostic.
    diagnostic = np.ones(bgt.shape + (3,), dtype=np.float32)
    diagnostic[bgt & ~matched_gt] = np.asarray([0.12, 0.12, 0.12])
    diagnostic[bpr & ~matched_pred] = np.asarray([0.88, 0.08, 0.08])
    diagnostic[matched_gt | matched_pred] = np.asarray([0.05, 0.62, 0.18])

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(14, 7),
        constrained_layout=True,
    )

    axes[0].imshow(
        layered_base,
        origin="lower",
        interpolation="nearest",
    )
    axes[0].imshow(
        pred_overlay,
        origin="lower",
        interpolation="nearest",
    )
    axes[0].set_title(
        "Literal layers: B_GT below, B_PRED above"
    )
    axes[0].legend(
        handles=[
            Patch(
                facecolor=(0.12, 0.12, 0.12),
                label="B_GT_t reference",
            ),
            Patch(
                facecolor=(0.90, 0.10, 0.10, 0.55),
                label="B_PRED_t prediction",
            ),
        ],
        loc="upper right",
        fontsize=8,
    )

    axes[1].imshow(
        diagnostic,
        origin="lower",
        interpolation="nearest",
    )

    # Draw faint links for accepted matched pairs when GT/pred endpoints differ.
    for pair in matching_010["pairs"]:
        gr = int(pair["gt_row"])
        gc = int(pair["gt_col"])
        pr = int(pair["pred_row"])
        pc = int(pair["pred_col"])
        if (
            r0 <= gr < r1
            and c0 <= gc < c1
            and r0 <= pr < r1
            and c0 <= pc < c1
            and (gr != pr or gc != pc)
        ):
            axes[1].plot(
                [gc - c0, pc - c0],
                [gr - r0, pr - r0],
                linewidth=0.45,
                alpha=0.30,
                color=(0.05, 0.62, 0.18),
            )

    axes[1].set_title(
        "Accepted matching @ 0.10 m"
    )
    axes[1].legend(
        handles=[
            Patch(
                facecolor=(0.05, 0.62, 0.18),
                label="matched endpoint",
            ),
            Patch(
                facecolor=(0.12, 0.12, 0.12),
                label="GT-only / unmatched",
            ),
            Patch(
                facecolor=(0.88, 0.08, 0.08),
                label="Pred-only / unmatched",
            ),
        ],
        loc="upper right",
        fontsize=8,
    )

    for ax in axes:
        ax.set_xticks([])
        ax.set_yticks([])

    fig.suptitle(
        "R002 Reference B — mpx_001 "
        f"decision {decision_id:06d}\n"
        f"GT={matching_010['gt_count']}, "
        f"Pred={matching_010['pred_count']}, "
        f"Matched@0.10={matching_010['matched']}, "
        f"P={matching_010['precision']:.3f}, "
        f"R={matching_010['recall']:.3f}, "
        f"F1={matching_010['f1']:.3f}"
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)

    return {
        "decision_id": decision_id,
        "file": output_path.name,
        "b_gt_count": int(matching_010["gt_count"]),
        "b_pred_count": int(matching_010["pred_count"]),
        "b_matched_0p10": int(matching_010["matched"]),
        "b_precision_0p10": float(matching_010["precision"]),
        "b_recall_0p10": float(matching_010["recall"]),
        "b_f1_0p10": float(matching_010["f1"]),
        "crop_r0": r0,
        "crop_r1": r1,
        "crop_c0": c0,
        "crop_c1": c1,
    }


def _parse_float(value: str) -> float:
    return float(value.strip())


def _float_same(a: float, b: float) -> bool:
    if math.isnan(a) and math.isnan(b):
        return True
    return math.isclose(a, b, rel_tol=0.0, abs_tol=1e-12)


def _verify_against_accepted(
    rendered_rows: list[dict],
    accepted_csv: Path,
) -> None:
    accepted_rows = r002._read_csv(accepted_csv)
    accepted_by_id = {
        int(row["decision_id"]): row
        for row in accepted_rows
    }

    if len(accepted_by_id) != EXPECTED_DECISIONS:
        raise AssertionError(
            "accepted R002 decision CSV does not contain exactly "
            f"{EXPECTED_DECISIONS} unique decisions"
        )

    for rendered in rendered_rows:
        decision_id = int(rendered["decision_id"])
        accepted = accepted_by_id.get(decision_id)
        if accepted is None:
            raise AssertionError(
                f"accepted R002 CSV missing decision {decision_id}"
            )

        for key in (
            "b_gt_count",
            "b_pred_count",
            "b_matched_0p10",
        ):
            if int(rendered[key]) != int(accepted[key]):
                raise AssertionError(
                    f"decision {decision_id}: {key} mismatch "
                    f"render={rendered[key]} accepted={accepted[key]}"
                )

        for key in (
            "b_precision_0p10",
            "b_recall_0p10",
            "b_f1_0p10",
        ):
            render_value = float(rendered[key])
            accepted_value = _parse_float(accepted[key])
            if not _float_same(render_value, accepted_value):
                raise AssertionError(
                    f"decision {decision_id}: {key} mismatch "
                    f"render={render_value} accepted={accepted_value}"
                )


def _write_index(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError("cannot write empty Reference-B visual index")
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_readme(
    path: Path,
    accepted_csv: Path | None,
) -> None:
    verification = (
        f"Verified against accepted first-score CSV: `{accepted_csv}`."
        if accepted_csv is not None
        else "No accepted first-score CSV was supplied for numeric parity verification."
    )

    path.write_text(
        f"""# R002 Reference-B visual audit — mpx_001

This directory is a **visualization-only** audit of the already accepted
R002 Reference-B semantics.

## Frozen sets shown

```text
B_GT_t   = GT_surface ∩ U_t
B_PRED_t = PredictedOccupiedBoundary_t ∩ U_t
```

The renderer imports the accepted projection, threshold, boundary extraction
and 0.10 m one-to-one matching implementation from
`r002_gt_semantics.py`. It does not redefine R002 metric semantics.

## Image layout

Each `decision_XXXXXX_reference_b.png` has two panels:

1. **Literal layers**
   - lower/reference layer: B_GT_t in dark gray/black;
   - upper/prediction layer: B_PRED_t in semi-transparent red.

2. **Accepted 0.10 m matching diagnostic**
   - green: GT/pred endpoints participating in the accepted one-to-one match;
   - dark gray/black: unmatched GT target;
   - red: unmatched predicted boundary;
   - faint green segment: matched GT/pred cells when their cells differ.

The right panel is only a visualization of the already-frozen matching.
The official quantitative Reference-B result remains the evaluator output.

## Scope

- run: `mpx_001` only;
- expected decision images: `35`;
- threshold: imported frozen value `{r002.PREDICTION_THRESHOLD}`;
- primary B tolerance: imported frozen value `{r002.PRIMARY_BOUNDARY_TOLERANCE_M} m`.

{verification}

See `index.csv` for per-decision support counts and accepted 0.10 m
precision/recall/F1 values used in the figure titles.
""",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    script_path = Path(__file__).resolve()
    mapex_lab_root = script_path.parents[2]

    parser = argparse.ArgumentParser(
        description=(
            "Render all 35 frozen R002 Reference-B overlays for mpx_001"
        )
    )
    parser.add_argument(
        "--run",
        default=r002.ALLOWED_RUN,
    )
    parser.add_argument(
        "--experiments-root",
        type=Path,
        default=mapex_lab_root / "experiments" / "mapex",
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=(
            mapex_lab_root
            / "ground_truth"
            / "new_room"
            / "generated"
            / "new_room_structural_gt_v2.npz"
        ),
    )
    parser.add_argument(
        "--connected-free",
        type=Path,
        default=(
            mapex_lab_root
            / "ground_truth"
            / "new_room"
            / "generated"
            / "new_room_connected_free_v2.npy"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            script_path.parent
            / "results"
            / "r002_b_vis_mpx_001_h012"
        ),
    )
    parser.add_argument(
        "--accepted-decisions-csv",
        type=Path,
        default=None,
        help=(
            "Optional accepted r002_decisions.csv. When supplied, the "
            "renderer hard-checks B counts and 0.10 m metrics decision-by-decision."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.run != r002.ALLOWED_RUN:
        raise ValueError(
            f"H012 is limited to {r002.ALLOWED_RUN}; refusing {args.run}"
        )

    run_dir = args.experiments_root.expanduser().resolve() / args.run
    output_dir = args.output_dir.expanduser().resolve()

    if output_dir.exists():
        if any(output_dir.iterdir()):
            raise FileExistsError(
                "refusing to overwrite existing non-empty output directory: "
                f"{output_dir}"
            )
    else:
        output_dir.mkdir(parents=True, exist_ok=False)

    gt = r002.gate_p.load_structural_gt(
        args.ground_truth.expanduser().resolve()
    )
    connected_free = np.asarray(
        np.load(
            args.connected_free.expanduser().resolve(),
            allow_pickle=False,
        ),
        dtype=bool,
    )
    if connected_free.shape != gt.shape:
        raise ValueError("connected-free mask shape does not match structural GT")

    decisions = sorted(
        r002._read_csv(run_dir / "decisions.csv"),
        key=lambda row: int(row["decision_id"]),
    )
    if len(decisions) != EXPECTED_DECISIONS:
        raise AssertionError(
            f"H012 expected {EXPECTED_DECISIONS} decisions; got {len(decisions)}"
        )
    decision_ids = [int(row["decision_id"]) for row in decisions]
    if len(set(decision_ids)) != EXPECTED_DECISIONS:
        raise AssertionError("mpx_001 decision IDs are not unique")

    metadata = json.loads(
        (run_dir / "metadata.json").read_text(encoding="utf-8")
    )
    observed_grids = [
        r002.gate_p.load_raw_grid(
            r002._resolve_run_path(run_dir, row["raw_map"])
        )
        for row in decisions
    ]
    warnings = r002.gate_p.validate_structural_gt_provenance(
        run_name=run_dir.name,
        metadata=metadata,
        structural_gt=gt,
        decision_grids=observed_grids,
    )
    for warning in warnings:
        print(f"[WARN] {run_dir.name}: {warning}")

    structural_occupied = gt.data > r002.GT_OCC_THRESHOLD
    gt_surface = structural_occupied & r002._adjacent8(connected_free)

    images_dir = output_dir / "images"
    rows: list[dict] = []

    for decision, observed in zip(decisions, observed_grids):
        decision_id = int(decision["decision_id"])

        projection = r002._canonical_projection(observed, gt)
        universe = projection["universe"]
        support = projection["support"]

        prediction = r002.gate_p.load_runtime_prediction(
            r002._resolve_run_path(run_dir, decision["mean_map"]),
            expected_grid=observed,
            expected_member="mean",
            expected_environment=(
                str(metadata.get("environment", "")) or None
            ),
        )
        pred_canvas = r002._canonical_prediction(
            prediction,
            projection,
            gt,
        )
        if not np.all(np.isfinite(pred_canvas[universe])):
            raise ValueError(
                f"decision {decision_id}: non-finite prediction in U_t"
            )

        # Reference B copied by composition from the accepted frozen helpers.
        hybrid_free = (
            projection["observed_free"]
            | (
                projection["unknown"]
                & (pred_canvas < r002.PREDICTION_THRESHOLD)
            )
        ) & support
        hybrid_occupied = (
            projection["observed_occupied"]
            | (
                projection["unknown"]
                & (pred_canvas >= r002.PREDICTION_THRESHOLD)
            )
        ) & support

        predicted_occupied_boundary = r002._occupied_boundary(
            hybrid_occupied,
            hybrid_free,
        )
        b_gt = gt_surface & universe
        b_pred = predicted_occupied_boundary & universe

        matching_010 = r002._boundary_matching(
            r002._coords(b_gt),
            r002._coords(b_pred),
            gt.resolution,
            r002.PRIMARY_BOUNDARY_TOLERANCE_M,
        )

        output_path = (
            images_dir
            / f"decision_{decision_id:06d}_reference_b.png"
        )
        row = _render_decision(
            output_path=output_path,
            decision_id=decision_id,
            projection=projection,
            b_gt=b_gt,
            b_pred=b_pred,
            matching_010=matching_010,
        )
        rows.append(row)

        print(
            f"[R002-B-VIS] decision {decision_id:03d}: "
            f"GT={row['b_gt_count']} "
            f"Pred={row['b_pred_count']} "
            f"Matched@0.10={row['b_matched_0p10']} "
            f"F1={row['b_f1_0p10']:.4f}"
        )

    image_files = sorted(images_dir.glob("decision_*_reference_b.png"))
    if len(image_files) != EXPECTED_DECISIONS:
        raise AssertionError(
            f"expected {EXPECTED_DECISIONS} rendered images; got {len(image_files)}"
        )

    accepted_csv: Path | None = None
    if args.accepted_decisions_csv is not None:
        accepted_csv = args.accepted_decisions_csv.expanduser().resolve()
        _verify_against_accepted(rows, accepted_csv)
        print(
            "[R002-B-VIS] accepted first-score B parity: PASS "
            f"({accepted_csv})"
        )

    _write_index(output_dir / "index.csv", rows)
    _write_readme(output_dir / "README.md", accepted_csv)

    manifest = {
        "schema_version": OUTPUT_SCHEMA,
        "run_id": args.run,
        "decision_count": EXPECTED_DECISIONS,
        "image_count": len(image_files),
        "prediction_threshold": r002.PREDICTION_THRESHOLD,
        "primary_boundary_tolerance_m": r002.PRIMARY_BOUNDARY_TOLERANCE_M,
        "accepted_decisions_csv": (
            str(accepted_csv) if accepted_csv is not None else None
        ),
        "accepted_parity_checked": accepted_csv is not None,
        "visualization_only": True,
        "images": [path.name for path in image_files],
        "provenance_warnings": warnings,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    print(f"[R002-B-VIS] wrote {len(image_files)} decision images")
    print(f"[R002-B-VIS] output_dir={output_dir}")
    print("[R002-B-VIS] visualization_only=True")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
