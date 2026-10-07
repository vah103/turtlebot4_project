"""Scientific V2 comparisons and a fixed-first-case action-checkpoint GIF."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from .render import image_map

BASE = Path(__file__).resolve().parent
METHODS = ["mapex", "uncertainty", "route_uncertainty", "route_error", "structural_v2"]
LABELS = {"mapex": "MapEx", "uncertainty": "Point uncertainty", "route_uncertainty": "Route uncertainty",
          "route_error": "Route error", "structural_v2": "Structural V2"}
COLORS = {"mapex": "#64748b", "uncertainty": "#e49b23", "route_uncertainty": "#0891b2",
          "route_error": "#9c4bb4", "structural_v2": "#2563eb"}


def comparison(output, phase):
    root = output/phase
    with (root/"branches.csv").open() as f:
        rows = list(csv.DictReader(f))
    layouts = sorted({r["layout"] for r in rows})
    fig, axes = plt.subplots(2, len(layouts), figsize=(14, 7), squeeze=False)
    width = .15
    for column, layout in enumerate(layouts):
        cases = sorted({r["case"] for r in rows if r["layout"] == layout}, key=lambda c: int(c.split("__warm")[1]))
        x = np.arange(len(cases))
        for i, method in enumerate(METHODS):
            for row, metric in enumerate(("reachable_mismatch_m2", "macro_iou")):
                values = [float(next(r[metric] for r in rows if r["case"] == case and r["method"] == method)) for case in cases]
                axes[row, column].bar(x+(i-2)*width, values, width, color=COLORS[method], label=LABELS[method])
        axes[0, column].set_title(layout.replace("kth_", "KTH "), fontsize=11)
        for row in (0, 1):
            ax = axes[row, column]
            ax.set_xticks(x, ["Warm state: "+case.split("__warm")[1]+" m" for case in cases], fontsize=9)
            ax.grid(axis="y", alpha=.2); ax.set_axisbelow(True)
            ax.spines[["right", "top"]].set_visible(False)
        axes[1, column].set_ylim(0, 1)
    axes[0, 0].set_ylabel("Reachability mismatch (m²)\nlower is better")
    axes[1, 0].set_ylabel("Macro IoU\nhigher is better")
    handles, labels = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(.5,.94), ncol=5, fontsize=9)
    fig.suptitle("V2: "+phase+" — common additional motion budget 8 m", fontsize=13)
    fig.subplots_adjust(top=.83, bottom=.08, hspace=.35, wspace=.28)
    fig.savefig(root/"comparison_v2.png", dpi=160); plt.close(fig)
    return rows


def case_plot(root, case, rows):
    layout = case.split("__")[0]
    with np.load(BASE/"assets"/(layout+".npz")) as z:
        truth = z["occupied"].copy()
    with np.load(root/"raw"/case/"initial.npz") as z:
        initial = {k:z[k].copy() for k in z.files}
    fig, axes = plt.subplots(2,4,figsize=(16,8))
    axes = axes.ravel()
    axes[0].imshow(~truth,cmap="gray",origin="lower"); axes[0].set_title("Ground truth: evaluation only")
    axes[1].imshow(image_map(initial["observed"]),origin="lower"); axes[1].set_title("Common initial observation")
    axes[2].imshow(image_map(initial["observed"],initial["mean"]),origin="lower"); axes[2].set_title("Initial real LaMa completion")
    for ax,method in zip(axes[3:],METHODS):
        with np.load(root/"raw"/case/method/"final.npz") as z:
            ax.imshow(image_map(z["observed"],z["mean"]),origin="lower")
        actions=json.loads((root/"raw"/case/method/"actions.json").read_text())
        for a in actions:
            path=np.asarray(a.get("path_taken",[]))
            if len(path):ax.plot(path[:,1],path[:,0],color=COLORS[method],linewidth=1.5)
            if a.get("hypothesis"):
                r,c=a["hypothesis"]["centre"];ax.scatter(c,r,color="#f97316",marker="x",s=25)
        record=next(r for r in rows if r["case"]==case and r["method"]==method)
        ax.set_title(LABELS[method]+"\nmismatch %.2f m²\nIoU %.3f; moved %.1f m" %
                     (float(record["reachable_mismatch_m2"]),float(record["macro_iou"]),float(record["distance_m"])),fontsize=10)
    for ax in axes:ax.set_xticks([]);ax.set_yticks([])
    fig.suptitle(case+" — paths in method colors; orange crosses: proposed gates",fontsize=12)
    fig.subplots_adjust(top=.89,bottom=.03,hspace=.35,wspace=.10)
    fig.savefig(root/(case+".png"),dpi=140);plt.close(fig)


def make_gif(root, case):
    methods=["uncertainty","route_error","structural_v2"]
    sequences={m:[root/"raw"/case/"initial.npz"]+sorted((root/"raw"/case/m).glob("frame_*.npz")) for m in methods}
    actions={m:json.loads((root/"raw"/case/m/"actions.json").read_text()) for m in methods}
    frames=[]
    for index in range(max(map(len,sequences.values()))):
        fig,axes=plt.subplots(3,1,figsize=(10,10))
        for ax,method in zip(axes,methods):
            sequence=sequences[method];checkpoint=min(index,len(sequence)-1)
            with np.load(sequence[checkpoint]) as z:
                ax.imshow(image_map(z["observed"],z["mean"]),origin="lower")
                r,c=z["pose"];ax.scatter(c,r,color=COLORS[method],s=20)
            distance=0.
            for a in actions[method][:checkpoint]:
                distance=a.get("distance_after_m",distance)
                path=np.asarray(a.get("path_taken",[]))
                if len(path):ax.plot(path[:,1],path[:,0],color=COLORS[method],linewidth=1.3)
            ax.set_title(LABELS[method]+" — %.1f m" % distance,fontsize=10)
            ax.set_xticks([]);ax.set_yticks([])
        fig.suptitle(case+" — fixed first new case; action checkpoints, not physical time",fontsize=11)
        fig.tight_layout();fig.canvas.draw()
        rgba=np.asarray(fig.canvas.buffer_rgba())
        frames.append(Image.fromarray(rgba[:,:,:3].copy()).convert("P",palette=Image.Palette.ADAPTIVE))
        plt.close(fig)
    frames[0].save(root/"pilot_v2_demo.gif",save_all=True,append_images=frames[1:],loop=0,
                   duration=[1600]+[1100]*(len(frames)-1))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=BASE/"results/pilot_v2")
    args=parser.parse_args()
    for phase in ("development","confirmation"):
        rows=comparison(args.output,phase)
        if phase=="confirmation":
            cases=sorted({r["case"] for r in rows})
            for case in cases:case_plot(args.output/phase,case,rows)
            make_gif(args.output/phase,cases[0])
    print("Rendered V2 development/confirmation comparison, six cases and fixed-first-case GIF")


if __name__ == "__main__":
    main()
