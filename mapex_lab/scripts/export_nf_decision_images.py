#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_canvas_npz(path: Path):
    with np.load(path) as bundle:
        data = np.asarray(bundle["data"], dtype=np.int16)
        resolution = float(np.asarray(bundle["resolution"]).reshape(-1)[0])
        origin_x = float(np.asarray(bundle["origin_x"]).reshape(-1)[0])
        origin_y = float(np.asarray(bundle["origin_y"]).reshape(-1)[0])
    return data, resolution, origin_x, origin_y


def to_display_image(data: np.ndarray) -> np.ndarray:
    # unknown = gray, free = white, occupied = black
    out = np.full(data.shape, 0.60, dtype=np.float32)
    out[(data >= 0) & (data <= 50)] = 1.00
    out[data > 50] = 0.00
    return out


def world_to_pixel(x: float, y: float, origin_x: float, origin_y: float, res: float):
    col = (x - origin_x) / res
    row = (y - origin_y) / res
    return col, row


def try_float(row: dict, keys: list[str]):
    for k in keys:
        if k in row and row[k] not in ("", None):
            try:
                return float(row[k])
            except Exception:
                pass
    return None


def load_candidates(path: Path):
    pts = []
    if not path.is_file():
        return pts

    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = try_float(row, ["x", "candidate_x", "frontier_x", "selected_x", "centroid_x"])
            y = try_float(row, ["y", "candidate_y", "frontier_y", "selected_y", "centroid_y"])
            if x is not None and y is not None:
                pts.append((x, y))
    return pts


def find_last_decision_canvas(run_dir: Path):
    decisions_root = run_dir / "decisions"
    decision_dirs = sorted(
        [p for p in decisions_root.iterdir() if p.is_dir() and p.name.startswith("policy_decision_")]
    )
    if not decision_dirs:
        return None
    return decision_dirs[-1] / "observed_map_canvas.npz"


def compute_common_crop_from_nf_final_maps(runs_root: Path, padding_px: int = 30):
    min_row = None
    max_row = None
    min_col = None
    max_col = None

    run_dirs = sorted([p for p in runs_root.iterdir() if p.is_dir() and p.name.startswith("nf_")])

    for run_dir in run_dirs:
        canvas_path = find_last_decision_canvas(run_dir)
        if canvas_path is None or not canvas_path.is_file():
            continue

        data, _, _, _ = load_canvas_npz(canvas_path)
        known = data >= 0
        rows, cols = np.where(known)
        if len(rows) == 0 or len(cols) == 0:
            continue

        r0, r1 = int(rows.min()), int(rows.max())
        c0, c1 = int(cols.min()), int(cols.max())

        min_row = r0 if min_row is None else min(min_row, r0)
        max_row = r1 if max_row is None else max(max_row, r1)
        min_col = c0 if min_col is None else min(min_col, c0)
        max_col = c1 if max_col is None else max(max_col, c1)

    if min_row is None:
        raise RuntimeError("Không tính được crop chung từ final maps.")

    crop = {
        "row_min": max(0, min_row - padding_px),
        "row_max": max_row + padding_px,
        "col_min": max(0, min_col - padding_px),
        "col_max": max_col + padding_px,
    }
    return crop


def save_crop_json(crop: dict, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(crop, indent=2), encoding="utf-8")


def load_crop_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def render_one(decision_dir: Path, out_dir: Path, crop: dict | None):
    decision_json = decision_dir / "decision.json"
    canvas_npz = decision_dir / "observed_map_canvas.npz"
    candidates_csv = decision_dir / "candidates.csv"

    if not decision_json.is_file() or not canvas_npz.is_file():
        return None

    meta = json.loads(decision_json.read_text(encoding="utf-8"))
    data, res, origin_x, origin_y = load_canvas_npz(canvas_npz)
    img = to_display_image(data)

    decision_id = int(meta["policy_decision_id"])
    sim_time_s = float(meta.get("sim_time_s", 0.0))
    robot_x = float(meta.get("robot_x", 0.0))
    robot_y = float(meta.get("robot_y", 0.0))
    selected_x = meta.get("selected_x")
    selected_y = meta.get("selected_y")
    outcome = str(meta.get("outcome", ""))
    selected_distance = meta.get("selected_distance_m")
    candidate_count = meta.get("candidate_count", "")

    fig = plt.figure(figsize=(10, 6.5))
    ax = fig.add_subplot(111)
    ax.imshow(img, cmap="gray", origin="lower", vmin=0.0, vmax=1.0)

    # candidates
    cand_pts = load_candidates(candidates_csv)
    if cand_pts:
        xs, ys = [], []
        for x, y in cand_pts:
            c, r = world_to_pixel(x, y, origin_x, origin_y, res)
            xs.append(c)
            ys.append(r)
        ax.scatter(xs, ys, s=12, marker="o", linewidths=0, alpha=0.9)

    # robot
    rc, rr = world_to_pixel(robot_x, robot_y, origin_x, origin_y, res)
    ax.scatter([rc], [rr], s=80, marker="o", edgecolors="black", linewidths=1.0)

    # selected frontier
    if selected_x not in ("", None) and selected_y not in ("", None):
        sc, sr = world_to_pixel(float(selected_x), float(selected_y), origin_x, origin_y, res)
        ax.scatter([sc], [sr], s=140, marker="*", edgecolors="black", linewidths=1.0)

    if crop is not None:
        ax.set_xlim(crop["col_min"], crop["col_max"])
        ax.set_ylim(crop["row_min"], crop["row_max"])

    run_name = decision_dir.parent.parent.name
    pretty_run = run_name.replace("_", " ").upper()
    ax.set_title(f"{pretty_run} — Decision {decision_id:06d}", fontsize=13)
    ax.set_xticks([])
    ax.set_yticks([])

    dist_text = f"{float(selected_distance):.2f} m" if selected_distance not in ("", None) else "N/A"
    footer = (
        f"t = {sim_time_s:.1f} s   |   "
        f"candidates = {candidate_count}   |   "
        f"selected distance = {dist_text}   |   "
        f"outcome = {outcome}"
    )
    fig.text(0.5, 0.02, footer, ha="center", va="bottom", fontsize=10)

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"decision_{decision_id:06d}.png"

    plt.tight_layout(rect=[0.01, 0.07, 0.99, 0.96])
    fig.savefig(
        out_path,
        dpi=180,
        facecolor="white",
    )
    plt.close(fig)
    return out_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", help="Ví dụ: experiments/nearest/nf_001")
    parser.add_argument("--out", required=True, help="Thư mục output PNG")
    parser.add_argument("--decision-ids", nargs="*", type=int, default=None)
    parser.add_argument("--crop-file", default="exported_images/NF_Decisions/nf_common_crop.json")
    parser.add_argument("--crop-runs-root", default="experiments/nearest")
    parser.add_argument("--recompute-crop", action="store_true")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    out_dir = Path(args.out).expanduser().resolve()
    crop_file = Path(args.crop_file).expanduser().resolve()
    crop_runs_root = Path(args.crop_runs_root).expanduser().resolve()

    if args.recompute_crop or not crop_file.is_file():
        crop = compute_common_crop_from_nf_final_maps(crop_runs_root, padding_px=30)
        save_crop_json(crop, crop_file)
        print(f"Saved common crop to: {crop_file}")
        print(json.dumps(crop, indent=2))
    else:
        crop = load_crop_json(crop_file)
        print(f"Loaded common crop from: {crop_file}")
        print(json.dumps(crop, indent=2))

    decisions_root = run_dir / "decisions"
    if not decisions_root.is_dir():
        raise FileNotFoundError(f"Không thấy thư mục decisions: {decisions_root}")

    decision_dirs = sorted(
        [p for p in decisions_root.iterdir() if p.is_dir() and p.name.startswith("policy_decision_")]
    )

    wanted = set(args.decision_ids) if args.decision_ids else None
    exported = []

    for d in decision_dirs:
        try:
            did = int(d.name.split("_")[-1])
        except Exception:
            continue
        if wanted is not None and did not in wanted:
            continue
        out_path = render_one(d, out_dir, crop)
        if out_path is not None:
            exported.append(out_path)

    print(f"Exported {len(exported)} images to {out_dir}")
    for p in exported[:10]:
        print(p)


if __name__ == "__main__":
    main()
