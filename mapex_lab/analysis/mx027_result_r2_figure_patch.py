#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
OUT=HERE/"mx027_exec_results"
sys.path.insert(0,str(HERE))
from mx027_ig_coverage_exec import plot_run  # figure implementation only

CORE=[
"MX027_IG_COVERAGE_PER_DECISION.csv",
"MX027_IG_PER_RUN.csv",
"MX027_COVERAGE_PER_RUN.csv",
"MX027_IG_COVERAGE_PROGRESS_BINS.csv",
"MX027_IG_COVERAGE_WHOLE_RUN_DIRECTION.csv",
"MX027_IG_COVERAGE_METRIC_DICTIONARY.json",
"MX027_IG_COVERAGE_SOURCE_PARITY.csv",
"MX027_FIVE_FAMILY_PER_DECISION_VIEW.csv",
]
PARENT_TECH="a1bb6d98af21d558b1323aaa8edeab53e15aba41"
QA_REVIEW="d21b32bf90792b0e3385de09a56eb57c4f46f5f2"

def sha256(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""): h.update(b)
    return h.hexdigest()

def read_csv(p):
    with Path(p).open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))

def main():
    manifest_path=OUT/"MX027_IG_COVERAGE_ARTIFACT_MANIFEST.json"
    report_path=OUT/"MX027_ANALYST_REPORT.md"
    prov_path=OUT/"MX027_R2_FIGURE_CORRECTION_PROVENANCE.json"

    manifest=json.loads(manifest_path.read_text(encoding="utf-8"))
    old_hash={a["path"]:a["sha256"] for a in manifest["artifacts"]}
    core_before={name:sha256(OUT/name) for name in CORE}
    for name in CORE:
        if old_hash.get(name)!=core_before[name]:
            raise RuntimeError("PRE_PATCH_CORE_HASH_MISMATCH:"+name)

    rows=read_csv(OUT/"MX027_IG_COVERAGE_PER_DECISION.csv")
    byrun={}
    for r in rows: byrun.setdefault(r["run_id"],[]).append(r)
    for rid in byrun:
        byrun[rid]=sorted(byrun[rid],key=lambda r:int(r["decision_index"]))
    if sorted(byrun)!=[f"mpx_{i:03d}" for i in range(1,11)]:
        raise RuntimeError("RUN_SET_MISMATCH")

    old_fig={f"figures/MX027_mpx_{i:03d}_FULL_TRAJECTORY.svg":old_hash[f"figures/MX027_mpx_{i:03d}_FULL_TRAJECTORY.svg"] for i in range(1,11)}
    for rid in sorted(byrun): plot_run(rid,byrun[rid])

    core_after={name:sha256(OUT/name) for name in CORE}
    if core_before!=core_after:
        bad=[k for k in CORE if core_before[k]!=core_after[k]]
        raise RuntimeError("CORE_CHANGED:"+repr(bad))

    new_fig={name:sha256(OUT/name) for name in old_fig}
    unchanged_fig=[name for name in old_fig if old_fig[name]==new_fig[name]]
    if unchanged_fig:
        raise RuntimeError("FIGURE_NOT_REGENERATED:"+repr(unchanged_fig))

    report=report_path.read_text(encoding="utf-8")
    note=(
      "\n## Bounded R2 figure correction\n"
      "- The 10 full-trajectory figures now place IG_selected and IG_visible_unknown_cells on separate subpanels with distinct y-axis labels/units.\n"
      "- Existing NA values remain plot gaps; Oracle-4 remains a vertical marker only; no smoothing or interpolation is introduced.\n"
      "- The eight frozen core data/dictionary artifacts remain byte-identical to result V1.\n"
    )
    if "## Bounded R2 figure correction" not in report:
        report=report.rstrip()+note+"\n"
        report_path.write_text(report,encoding="utf-8")

    provenance={
      "schema":"mx027_bounded_result_r2_figure_correction_v1",
      "task":"MX027",
      "parent_technical_result":PARENT_TECH,
      "qa_review_commit":QA_REVIEW,
      "scope":"figure-only correction",
      "changes":{
        "plot_contract":"IG_selected and IG_visible_unknown_cells separated into distinct subpanels",
        "figure_count":10,
        "report_note_added":True,
        "manifest_refreshed":True
      },
      "unchanged_core":{
        "artifact_count":len(CORE),
        "all_sha256_unchanged":True,
        "artifacts":[{"path":n,"sha256":core_after[n]} for n in CORE]
      },
      "guards":{
        "source_reextract":False,
        "model_rerun":False,
        "simulation_rerun":False,
        "scientific_reanalysis":False,
        "retuning":False,
        "interpolation":False,
        "smoothing":False,
        "stop_rule":False,
        "composite":False,
        "winner":False,
        "new_correlation":False
      }
    }
    prov_path.write_text(json.dumps(provenance,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    manifest["status"]="COMPLETE_PENDING_FOCUSED_INDEPENDENT_RESULT_QA_R2"
    manifest["bounded_result_revision"]={
      "revision":"R2",
      "parent_technical_result":PARENT_TECH,
      "qa_review_commit":QA_REVIEW,
      "finding_closed":"MX027-RESULT-R1",
      "scope":"FIGURE_ONLY"
    }
    paths={a["path"] for a in manifest["artifacts"]}
    if prov_path.name not in paths:
        manifest["artifacts"].append({"path":prov_path.name,"sha256":"","size":0})
    for a in manifest["artifacts"]:
        p=OUT/a["path"]
        a["sha256"]=sha256(p); a["size"]=p.stat().st_size
    manifest_path.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    # Final core guard after report/manifest/provenance changes.
    for name in CORE:
        if sha256(OUT/name)!=core_before[name]:
            raise RuntimeError("POST_PATCH_CORE_HASH_MISMATCH:"+name)

    print(json.dumps({
      "core_unchanged":len(CORE),
      "figures_regenerated":10,
      "old_vs_new_figure_hash_changed":10,
      "report_updated":True,
      "manifest_updated":True
    },indent=2,sort_keys=True))

if __name__=="__main__": main()
