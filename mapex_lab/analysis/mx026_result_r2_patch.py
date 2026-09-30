#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, json, math
from pathlib import Path

LAB=Path(__file__).resolve().parents[1]
OUT=LAB/"analysis"/"mx026_exec_results"
MASTER=OUT/"MX026_PUR_PER_DECISION.csv"
BOUNDARY=OUT/"MX026_P_BOUNDARY_AUDIT.csv"
PER_RUN=OUT/"MX026_P_PER_RUN.csv"
REPORT=OUT/"MX026_ANALYST_REPORT.md"
MANIFEST=OUT/"MX026_PUR_ARTIFACT_MANIFEST.json"
PROV=OUT/"MX026_R2_CORRECTION_PROVENANCE.json"
TARGETS={("mpx_002","29"),("mpx_002","30")}
QA_REVIEW="99cb26e9f7f96ba9fb9cbb5af28fb2098d653a26"
V1_TECH="d39fcf574ddf59339d3f40db498071193ad098d6"

def read_csv(p):
    with p.open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rows):
    rows=list(rows); fields=list(rows[0].keys())
    with p.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def fin(v):
    try:x=float(v)
    except:return math.nan
    return x if math.isfinite(x) else math.nan
def med(vals):
    vals=sorted(float(v) for v in vals if math.isfinite(float(v)))
    n=len(vals)
    if not n:return math.nan
    return vals[n//2] if n%2 else (vals[n//2-1]+vals[n//2])/2.0
def sha256(p):
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()

def main():
    original_manifest=json.loads(MANIFEST.read_text(encoding="utf-8"))
    original_hashes={a["path"]:a["sha256"] for a in original_manifest["artifacts"]}

    master=read_csv(MASTER)
    bad_bal=set();bad_macro=set()
    for r in master:
        if r["P_later_observed_evaluable"]!="1":continue
        key=(r["run_id"],r["decision_id"])
        fr,orr=fin(r["P_LATER_FreeRecall"]),fin(r["P_LATER_OccupiedRecall"])
        fi,oi=fin(r["P_LATER_FreeIoU"]),fin(r["P_LATER_OccupiedIoU"])
        br,mi=fin(r["P_LATER_BalancedRecall"]),fin(r["P_LATER_MacroIoU"])
        if (not math.isfinite(fr) or not math.isfinite(orr)) and math.isfinite(br):bad_bal.add(key)
        if (not math.isfinite(fi) or not math.isfinite(oi)) and math.isfinite(mi):bad_macro.add(key)
    if bad_bal!=TARGETS or bad_macro!=TARGETS:
        raise RuntimeError("UNEXPECTED_MX026_R1_SCOPE:"+repr((bad_bal,bad_macro)))

    changed_master=[]
    for r in master:
        key=(r["run_id"],r["decision_id"])
        if key in TARGETS:
            if r["P_LATER_TF"]!="0" or fin(r["P_LATER_OccupiedRecall"])!=1.0 or fin(r["P_LATER_OccupiedIoU"])!=1.0:
                raise RuntimeError("TARGET_SEMANTICS_PRECONDITION_FAIL:"+repr(key))
            r["P_LATER_BalancedRecall"]="nan"
            r["P_LATER_MacroIoU"]="nan"
            changed_master.append(key)
    if set(changed_master)!=TARGETS:raise RuntimeError("MASTER_TARGET_SET_FAIL")
    write_csv(MASTER,master)

    boundary=read_csv(BOUNDARY);changed_boundary=[]
    for r in boundary:
        key=(r["run_id"],r["decision_id"])
        if key in TARGETS and r["domain"]=="Q1_LATER_OBSERVED":
            if r["primary_FreeRecall"]!="nan" or r["primary_FreeIoU"]!="nan" or fin(r["primary_OccupiedRecall"])!=1.0 or fin(r["primary_OccupiedIoU"])!=1.0:
                raise RuntimeError("BOUNDARY_PRECONDITION_FAIL:"+repr(key))
            r["primary_BalancedRecall"]="nan"
            r["primary_MacroIoU"]="nan"
            changed_boundary.append(key)
    if set(changed_boundary)!=TARGETS:raise RuntimeError("BOUNDARY_TARGET_SET_FAIL")
    write_csv(BOUNDARY,boundary)

    # Derive only the two affected mpx_002 later-observed per-run medians from corrected master.
    mpx2=[r for r in master if r["run_id"]=="mpx_002" and r["P_later_observed_evaluable"]=="1"]
    bal=med([fin(r["P_LATER_BalancedRecall"]) for r in mpx2 if math.isfinite(fin(r["P_LATER_BalancedRecall"]))])
    mac=med([fin(r["P_LATER_MacroIoU"]) for r in mpx2 if math.isfinite(fin(r["P_LATER_MacroIoU"]))])
    if round(bal,9)!=0.852542492 or round(mac,9)!=0.727332365:
        raise RuntimeError("QA_EXPECTED_MEDIAN_FAIL:"+repr((bal,mac)))

    per=read_csv(PER_RUN);hit=0
    for r in per:
        if r["run_id"]=="mpx_002":
            r["P_LATER_BalancedRecall_median"]=str(bal)
            r["P_LATER_MacroIoU_median"]=str(mac)
            hit+=1
    if hit!=1:raise RuntimeError("MPX002_PER_RUN_ROW_FAIL")
    write_csv(PER_RUN,per)

    text=REPORT.read_text(encoding="utf-8")
    caveat=(
      "- Structural-GT P at each decision is evaluated on that decision's scoreable unknown-space population.\n"
      "- That structural evaluation population changes as exploration proceeds.\n"
      "- Therefore first-to-final structural-P trends describe the evolving decision-time evaluation domain and are not, by themselves, a fixed-population longitudinal accuracy experiment.\n"
    )
    anchor="## 5. Interpretation boundary\n"
    if caveat not in text:
        if anchor not in text:raise RuntimeError("REPORT_ANCHOR_MISSING")
        text=text.replace(anchor,anchor+caveat,1)
    REPORT.write_text(text,encoding="utf-8")

    provenance={
      "schema":"mx026_bounded_result_r2_correction_v1",
      "task":"MX026",
      "parent_technical_result":V1_TECH,
      "qa_review_commit":QA_REVIEW,
      "scope":"MX026-R1/MX026-R2 only",
      "model_inference_rerun":False,
      "simulation_rerun":False,
      "topology_rerun":False,
      "U_recomputed":False,
      "R_recomputed":False,
      "threshold_retuned":False,
      "changed_semantic_rows":{
        "master":[{"run_id":"mpx_002","decision_id":29},{"run_id":"mpx_002","decision_id":30}],
        "boundary_audit":[{"run_id":"mpx_002","decision_id":29,"domain":"Q1_LATER_OBSERVED"},{"run_id":"mpx_002","decision_id":30,"domain":"Q1_LATER_OBSERVED"}]
      },
      "corrected_values":{
        "mpx_002_d29":{"P_LATER_BalancedRecall":"NA","P_LATER_MacroIoU":"NA"},
        "mpx_002_d30":{"P_LATER_BalancedRecall":"NA","P_LATER_MacroIoU":"NA"},
        "mpx_002_later_per_run_median":{"BalancedRecall":bal,"MacroIoU":mac}
      },
      "report_caveat_added":True,
      "core_science_retained":["60 pairwise rows","90 whole-run direction rows","1650 progress-bin rows","U outputs","R outputs","10 full-run plots","p==0.5 counts"],
      "expected_changed_artifacts":["MX026_PUR_PER_DECISION.csv","MX026_P_BOUNDARY_AUDIT.csv","MX026_P_PER_RUN.csv","MX026_ANALYST_REPORT.md","MX026_PUR_ARTIFACT_MANIFEST.json","MX026_R2_CORRECTION_PROVENANCE.json"]
    }
    PROV.write_text(json.dumps(provenance,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    manifest=json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest["status"]="COMPLETE_PENDING_INDEPENDENT_RESULT_QA_R2"
    manifest["bounded_result_revision"]={
      "revision":"R2",
      "qa_review_commit":QA_REVIEW,
      "findings_closed":["MX026-R1","MX026-R2"],
      "parent_technical_result":V1_TECH
    }
    found={a["path"] for a in manifest["artifacts"]}
    if PROV.name not in found:
        manifest["artifacts"].append({"path":PROV.name,"sha256":"","size":0})
    for a in manifest["artifacts"]:
        p=OUT/a["path"]
        a["sha256"]=sha256(p);a["size"]=p.stat().st_size
    MANIFEST.write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8")

    # Assert every originally listed artifact outside the four R1/R2 content files remains byte-identical.
    allowed={"MX026_PUR_PER_DECISION.csv","MX026_P_BOUNDARY_AUDIT.csv","MX026_P_PER_RUN.csv","MX026_ANALYST_REPORT.md"}
    changed_unexpected=[]
    for path,oldhash in original_hashes.items():
        if path in allowed:continue
        if sha256(OUT/path)!=oldhash:changed_unexpected.append(path)
    if changed_unexpected:raise RuntimeError("UNEXPECTED_CORE_ARTIFACT_CHANGE:"+repr(changed_unexpected))

    print(json.dumps({
      "changed_master":[list(x) for x in sorted(TARGETS)],
      "changed_boundary":[list(x) for x in sorted(TARGETS)],
      "corrected_medians":{"BalancedRecall":bal,"MacroIoU":mac},
      "unchanged_core_artifacts_verified":len(original_hashes)-len(allowed),
      "report_caveat_added":True
    },indent=2,sort_keys=True))

if __name__=="__main__":main()
