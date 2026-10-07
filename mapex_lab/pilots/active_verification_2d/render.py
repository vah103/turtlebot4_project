"""Render paired pilot figures, one selected trajectory GIF, and summary HTML."""
from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

BASE = Path(__file__).resolve().parent
COLORS = {"mapex": "#64748b", "uncertainty": "#e49b23", "structural": "#2563eb"}
LABELS = {"mapex": "MapEx", "uncertainty": "Uncertainty", "structural": "Structural verification"}


def image_map(observed, mean=None):
    a = observed if mean is None else np.where(observed == .5, mean, observed)
    pixels = np.clip((1-a)*255, 0, 255).astype(np.uint8)
    rgb = np.repeat(pixels[:, :, None], 3, axis=2)
    rgb[observed == .5] = np.array([186, 195, 204]) if mean is None else rgb[observed == .5]
    return rgb


def summary_plot(rows, output):
    cases = sorted({r["case"] for r in rows})
    x = np.arange(len(cases))
    width = .24
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    for i, method in enumerate(["mapex", "uncertainty", "structural"]):
        by_case = {r["case"]: r for r in rows if r["method"] == method}
        for axis, key in zip(axes, ["reachable_mismatch_m2", "macro_iou"]):
            axis.bar(x+(i-1)*width, [float(by_case[c][key]) for c in cases], width,
                     color=COLORS[method], label=LABELS[method])
        for j, case in enumerate(cases):
            if by_case[case]["termination"] != "DISTANCE_BUDGET":
                axes[0].text(j+(i-1)*width, float(by_case[case]["reachable_mismatch_m2"]), "*", ha="center")
    labels = [c.replace("kth_", "KTH ").replace("new_room", "New Room").replace("__warm", "\nstart @ ")+" m" for c in cases]
    for axis in axes:
        axis.set_xticks(x, labels, fontsize=9)
        axis.spines[["right", "top"]].set_visible(False)
        axis.grid(axis="y", alpha=.2)
        axis.set_axisbelow(True)
    axes[0].set_ylabel("Reachability mismatch (m²) — lower is better")
    axes[1].set_ylabel("Macro IoU — higher is better")
    axes[1].set_ylim(0, 1)
    axes[0].legend(fontsize=8)
    fig.suptitle("Exploratory 2D pilot: real LaMa predictions, additional motion budget ≤ 8 m\n3 layouts × 2 warm states; * = stopped before using the full budget", fontsize=12)
    fig.tight_layout()
    fig.savefig(output/"comparison.png", dpi=160)
    plt.close(fig)


def case_plot(case, rows, output):
    layout = case.split("__")[0]
    with np.load(BASE/"assets"/(layout+".npz")) as z:
        occupied, domain = z["occupied"], z["domain"]
    initial = np.load(output/"raw"/case/"initial.npz")
    fig, axes = plt.subplots(2, 3, figsize=(12, 7))
    methods = ["mapex", "uncertainty", "structural"]
    axes[0, 0].imshow(~occupied, cmap="gray", origin="lower")
    axes[0, 0].set_title("Ground truth — evaluation only")
    axes[0, 1].imshow(image_map(initial["observed"]), origin="lower")
    axes[0, 1].set_title("Common initial observation")
    axes[0, 2].imshow(image_map(initial["observed"], initial["mean"]), origin="lower")
    axes[0, 2].set_title("Initial LaMa completion")
    for column, method in enumerate(methods):
        final = np.load(output/"raw"/case/method/"final.npz")
        ax = axes[1, column]
        ax.imshow(image_map(final["observed"], final["mean"]), origin="lower")
        actions = json.loads((output/"raw"/case/method/"actions.json").read_text())
        for action in actions:
            path = np.asarray(action.get("path_taken", []))
            if len(path):
                ax.plot(path[:, 1], path[:, 0], color=COLORS[method], linewidth=1.3)
            hypothesis = action.get("hypothesis")
            if hypothesis:
                r, c = hypothesis["centre"]
                ax.scatter(c, r, marker="x", color="#f97316", s=35)
        row = next(r for r in rows if r["case"] == case and r["method"] == method)
        ax.set_title("%s\nmismatch %.2f m²; IoU %.3f; moved %.1f m" %
                     (LABELS[method], float(row["reachable_mismatch_m2"]), float(row["macro_iou"]), float(row["distance_m"])), fontsize=10)
    for ax in axes.ravel():
        ax.set_xticks([])
        ax.set_yticks([])
    fig.suptitle(case+" — blue/orange/gray: paths; orange crosses: proposed gates", fontsize=12)
    fig.tight_layout()
    fig.savefig(output/(case+".png"), dpi=150)
    plt.close(fig)


def make_gif(case, output):
    methods = ["mapex", "uncertainty", "structural"]
    initial = np.load(output/"raw"/case/"initial.npz")
    sequences, actions = {}, {}
    for method in methods:
        folder = output/"raw"/case/method
        sequences[method] = [output/"raw"/case/"initial.npz"] + sorted(folder.glob("frame_*.npz"))
        actions[method] = json.loads((folder/"actions.json").read_text())
    frames = []
    for index in range(max(map(len, sequences.values()))):
        fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
        for ax, method in zip(axes, methods):
            sequence = sequences[method]
            with np.load(sequence[min(index, len(sequence)-1)]) as z:
                ax.imshow(image_map(z["observed"], z["mean"]), origin="lower")
                r, c = z["pose"]
                ax.scatter(c, r, color=COLORS[method], s=25)
            distance = 0.0
            for action in actions[method][:index]:
                distance = action.get("distance_after_m", distance)
                path = np.asarray(action.get("path_taken", []))
                if len(path):
                    ax.plot(path[:, 1], path[:, 0], color=COLORS[method], linewidth=1)
                if action.get("hypothesis"):
                    rr, cc = action["hypothesis"]["centre"]
                    ax.scatter(cc, rr, color="#f97316", marker="x", s=40)
            ax.set_title(LABELS[method]+" — %.1f m" % distance, fontsize=10)
            ax.set_xticks([])
            ax.set_yticks([])
        fig.suptitle(case+" — action checkpoints, not synchronized physical time", fontsize=10)
        fig.tight_layout()
        fig.canvas.draw()
        rgba = np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba[:, :, :3].copy()).convert("P", palette=Image.Palette.ADAPTIVE))
        plt.close(fig)
    frames[0].save(output/"pilot_demo.gif", save_all=True, append_images=frames[1:],
                   duration=[1600]+[1100]*(len(frames)-1), loop=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v1")
    args = parser.parse_args()
    output = args.output
    with (output/"branches.csv").open() as f:
        rows = list(csv.DictReader(f))
    summary_plot(rows, output)
    cases = sorted({r["case"] for r in rows})
    for case in cases:
        case_plot(case, rows, output)
    # Fixed first case, not selected for looking successful.
    selected = cases[0]
    make_gif(selected, output)
    table = "".join("<tr>"+"".join("<td>"+html.escape(str(r[k]))+"</td>" for k in
                          ["case", "method", "distance_m", "termination", "reachable_mismatch_m2", "macro_iou", "verify_decisions"])+"</tr>" for r in rows)
    (output/"index.html").write_text("""<!doctype html><meta charset="utf-8"><title>2D verification pilot</title>
<style>body{font:16px system-ui;max-width:1100px;margin:30px auto;padding:0 20px;color:#18212f}img{max-width:100%}table{border-collapse:collapse;font-size:13px}td,th{padding:8px;border-bottom:1px solid #ddd}</style>
<h1>Exploratory 2D verification pilot</h1><p>Real LaMa ensemble. Ideal 2D sensing and localization. No operational STOP policy. Six states from three layouts; this is development evidence.</p>
<img src="comparison.png"><h2>Action checkpoints</h2><img src="pilot_demo.gif"><h2>All branches</h2>
<table><tr><th>Case</th><th>Method</th><th>Distance</th><th>Termination</th><th>Mismatch m²</th><th>Macro IoU</th><th>Verify actions</th></tr>"""+table+"</table>"+
"".join('<h2>'+html.escape(case)+'</h2><img src="'+case+'.png">' for case in cases))
    print("Rendered", len(cases), "case figures, comparison, GIF, HTML")


if __name__ == "__main__":
    main()
