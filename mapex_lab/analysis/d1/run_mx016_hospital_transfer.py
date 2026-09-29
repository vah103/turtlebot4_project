#!/usr/bin/env python3
"""Run separately sealed MX016 Phase A or truth-side Phase B."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from mapex_lab.analysis.d1.mx016_hospital_transfer import run_phase_a, run_phase_b, write_artifact_manifest

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_RUN=ROOT/"mapex_lab/experiments/mapex/hpx_001"
DEFAULT_OUTPUT=ROOT/"mapex_lab/analysis/d1/results/mx016_hospital_transfer_v1"
DEFAULT_GT=ROOT/"mapex_lab/ground_truth/hospital/generated/hospital_structural_gt_v1.npz"
DEFAULT_ROI=ROOT/"mapex_lab/ground_truth/hospital/generated/hospital_connected_free_v1.npy"

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("phase",choices=("phase-a","phase-b","manifest"))
    parser.add_argument("--run-dir",type=Path,default=DEFAULT_RUN); parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument("--gt",type=Path,default=DEFAULT_GT); parser.add_argument("--roi",type=Path,default=DEFAULT_ROI)
    args=parser.parse_args()
    if args.phase=="phase-a":
        value=run_phase_a(args.run_dir,args.output,ROOT/"mapex_lab/analysis/d1/mx013_online_stop.py",ROOT/"mapex_lab/analysis/d1/mx016_hospital_transfer.py")
    elif args.phase=="phase-b": value=run_phase_b(args.run_dir,args.output,args.gt,args.roi)
    else: value=write_artifact_manifest(args.output)
    print(json.dumps(value,sort_keys=True))

if __name__=="__main__": main()
