#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,hashlib,json,math
from pathlib import Path

RUNS=("hpx_001","hpx_002","hpx_003","hpx_004","hpx_005")
COUNTS={"hpx_001":56,"hpx_002":51,"hpx_003":49,"hpx_004":52,"hpx_005":50}
GT_SHA="080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1"
ROI_SHA="05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
SEM_SHA="d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea"
METHOD_COMMIT="a94055cb793a8096a1152fc20f3ac58d3a20bcba"
METHOD_BLOB="62796c55eb3cb810eb2865d6c9d56d9c4fb0bfab"
METHOD_QA="098e2ca7ce15f456cc7918e036140a41577499f9"
PM_HANDOFF="8a1707e421268b594e437031eeb15caf5bc060e3"
COLLECTION="1e622f03b8cc90dcb1c431e459e773fa6de26374"

def sha(p:Path):
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1<<20),b""):h.update(b)
    return h.hexdigest()
def rows(p):
    with p.open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def wcsv(p,rs):
    rs=list(rs); keys=[]; seen=set()
    for r in rs:
        for k in r:
            if k not in seen:seen.add(k);keys.append(k)
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rs)
def wjson(p,o):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(o,indent=2,sort_keys=True)+"\n",encoding="utf-8")
def b(v):return int(bool(v))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--source-root",required=True)
    ap.add_argument("--out",required=True)
    ap.add_argument("--hpx001-exact-source-root",required=True)
    a=ap.parse_args()
    root=Path(a.source_root).resolve(); out=Path(a.out).resolve()
    exact1=Path(a.hpx001_exact_source_root).resolve()
    exps=root/"mapex_lab/experiments/mapex"
    batch=root/"mapex_lab/analysis/d1/results/mx029_hospital_5run"
    gt=root/"mapex_lab/analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2/hospital_structural_gt_v2.npz"
    roi=gt.with_name("hospital_connected_free_v2.npy")
    sem=gt.with_name("gt_v2_semantic_manifest.canonical.json")
    align=gt.with_name("alignment_validation.json")
    run_manifest=json.load(open(batch/"run_manifest.json"))
    preflight=json.load(open(batch/"preflight.json"))
    alignment=json.load(open(align))

    hard=[]
    if sha(gt)!=GT_SHA:hard.append("GT_SHA_MISMATCH")
    if sha(roi)!=ROI_SHA:hard.append("ROI_SHA_MISMATCH")
    if sha(sem)!=SEM_SHA:hard.append("SEMANTIC_SHA_MISMATCH")
    if not alignment.get("all_decisions_pass") or alignment.get("decision_count")!=56:
        hard.append("HPX001_GT_V2_ALIGNMENT_NOT_56_56_PASS")

    inventory=[]; avail=[]; gta=[]
    manifest_runs=[]
    for run in RUNS:
        rp=exps/run; meta_p=rp/"metadata.json"; summ_p=rp/"summary.json"
        meta=json.load(open(meta_p)); summ=json.load(open(summ_p))
        ds=rows(rp/"decisions.csv"); ps=rows(rp/"policy_decisions.csv")
        N=COUNTS[run]
        pred_counts={m:len(list((rp/"predictions").glob(f"decision_*_{m}.npz"))) for m in ("g1","g2","g3","mean","variance")}
        raw_n=len(list((rp/"decision_maps").glob("decision_*_raw.npz")))
        canvas_n=len(list((rp/"decision_maps").glob("decision_*_canvas.npz")))
        link_ok=len(ds)==N and len(ps)==N and [int(x["decision_id"]) for x in ds]==list(range(1,N+1)) and all(int(x["mapex_policy_decision_id"])==int(x["decision_id"]) for x in ds)
        expected_meta = (preflight.get("run",{}).get("metadata_sha256") if run=="hpx_001" else run_manifest["runs"][run]["inventory"]["metadata_sha256"])
        expected_summary = (preflight.get("run",{}).get("summary_sha256") if run=="hpx_001" else run_manifest["runs"][run]["inventory"]["summary_sha256"])
        # preflight schemas differ; fail only on a present expected value.
        meta_match=(expected_meta is None or sha(meta_p)==expected_meta)
        summary_match=(expected_summary is None or sha(summ_p)==expected_summary)
        provenance="LEGACY_RETROSPECTIVE_GT_V2" if run=="hpx_001" else "PROSPECTIVE_GT_V2"
        ok=(len(ds)==N and len(ps)==N and raw_n==N and canvas_n==N and all(v==N for v in pred_counts.values()) and link_ok and meta_match and summary_match and summ.get("termination_reason")=="exploration_complete")
        if not ok: hard.append(f"{run}:INVENTORY_OR_LINKAGE")
        inventory.append({
            "run_id":run,"provenance_stratum":provenance,"accepted_decision_count":N,
            "decisions_csv_n":len(ds),"policy_decisions_csv_n":len(ps),"raw_map_n":raw_n,"canvas_map_n":canvas_n,
            **{f"{k}_prediction_n":v for k,v in pred_counts.items()},
            "metadata_sha256":sha(meta_p),"metadata_hash_match":b(meta_match),
            "summary_sha256":sha(summ_p),"summary_hash_match":b(summary_match),
            "termination_reason":summ.get("termination_reason",""),"decision_policy_linkage_pass":b(link_ok),
            "git_commit":meta.get("git_commit",""),"git_dirty_at_recorder_start":b(meta.get("git_dirty_at_recorder_start")),
            "inventory_pass":b(ok)
        })
        for kind,pat in [("raw","decision_*_raw.npz"),("canvas","decision_*_canvas.npz")]:
            for p in sorted((rp/"decision_maps").glob(pat)):
                pass
        avail.append({
            "run_id":run,"decision_count":N,"raw_n":raw_n,"canvas_n":canvas_n,
            **{f"{k}_n":v for k,v in pred_counts.items()},
            "all_required_prediction_counts_match":b(all(v==N for v in pred_counts.values())),
            "trajectory_present":b((rp/"trajectory.csv").is_file()),
            "decisions_present":b((rp/"decisions.csv").is_file()),
            "policy_decisions_present":b((rp/"policy_decisions.csv").is_file()),
            "artifact_availability_pass":b(ok)
        })
        if run=="hpx_001":
            gt_class="LEGACY_RETROSPECTIVE_GT_V2_ACCEPTED_MX018_ALIGNMENT"
            gt_hash_bound=1; roi_hash_bound=1; alignment_pass=b(alignment.get("all_decisions_pass") and alignment.get("decision_count")==56)
            evidence="MX018 gt_v2/alignment_validation.json; 56/56 PASS"
        else:
            gt_class="PROSPECTIVE_GT_V2_BOUND_BEFORE_EXECUTION"
            gt_hash_bound=b(meta.get("structural_ground_truth_sha256")==GT_SHA)
            roi_hash_bound=b(meta.get("evaluation_roi_sha256")==ROI_SHA)
            alignment_pass=b(meta.get("fixed_canvas_id")=="hospital_canvas_v1" and abs(float(meta.get("fixed_canvas_resolution_m",0))-0.05)<1e-12)
            evidence="MX029 metadata + run_manifest pre-execution GT-v2 binding"
        gt_ok=gt_hash_bound and roi_hash_bound and alignment_pass
        if not gt_ok:hard.append(f"{run}:GT_PROVENANCE_ALIGNMENT")
        gta.append({"run_id":run,"provenance_stratum":provenance,"gt_class":gt_class,"gt_sha256":GT_SHA,"gt_hash_bound":gt_hash_bound,
                    "roi_sha256":ROI_SHA,"roi_hash_bound":roi_hash_bound,"semantic_digest":SEM_SHA,
                    "fixed_canvas_id":"hospital_canvas_v1","fixed_canvas_resolution_m":0.05,
                    "alignment_pass":alignment_pass,"evidence_ref":evidence,"gt_provenance_pass":b(gt_ok)})
        manifest_runs.append({
            "run_id":run,"provenance_stratum":provenance,"decision_count":N,
            "metadata_sha256":sha(meta_p),"summary_sha256":sha(summ_p),
            "git_commit":meta.get("git_commit"),"git_dirty_at_recorder_start":bool(meta.get("git_dirty_at_recorder_start")),
            "config_sha256":meta.get("config_sha256",{}),
            "ensemble_checkpoints":meta.get("ensemble_checkpoints",[]),
            "termination_reason":summ.get("termination_reason"),
            "raw_count":raw_n,"canvas_count":canvas_n,"prediction_counts":pred_counts
        })

    # Acquisition-exact source candidates. hpx_001 has three recovered historic byte versions.
    common={
      "mapex_profiled_evaluator":root/"mapex_lab/scripts/evaluate_mapex_profiled.py",
      "mapex_policy":root/"mapex_lab/scripts/mapex.py",
      "mapex_lama_bridge":root/"mapex_lab/scripts/mapex_lama_bridge.py",
      "mapex_lama_worker":root/"mapex_lab/scripts/mapex_lama_worker.py",
      "nf_basic_shared_execution":root/"mapex_lab/scripts/nf_basic.py",
      "mapex_config":root/"mapex_lab/config/mapex.yaml",
      "runtime_launch":root/"mapex_lab/launch/slam.launch.py",
      "nav2_override":root/"mapex_lab/config/nav2.yaml",
    }
    prospective={
      "mapex_run":root/"mapex_lab/scripts/mapex_run.py",
      "mapex_evaluator":root/"mapex_lab/scripts/evaluate_mapex_run.py",
      "nf_run_recorder_base":root/"mapex_lab/scripts/nf_run.py",
    }
    legacy={
      "mapex_run":exact1/"mapex_run.py",
      "mapex_evaluator":exact1/"evaluate_mapex_run.py",
      "nf_run_recorder_base":exact1/"nf_run.py",
    }
    surface={
      "mapex_run":"raw/fixed-canvas save; prediction-array save metadata/support geometry; decision/policy linkage; V1 source bundle",
      "mapex_evaluator":"canonical Hospital canvas/evaluation projection semantics",
      "nf_run_recorder_base":"recorder lifecycle, decision/policy linkage, ordinary-run I/O semantics",
      "mapex_profiled_evaluator":"profiled evaluator identity (not used to regenerate MX065 inputs)",
      "mapex_policy":"policy identity; no future/GT feature generation",
      "mapex_lama_bridge":"prediction bridge identity",
      "mapex_lama_worker":"prediction worker identity",
      "nf_basic_shared_execution":"shared ordinary execution identity",
      "mapex_config":"MapEx configuration identity",
      "runtime_mapex":"per-run runtime MapEx configuration identity",
      "runtime_launch":"runtime launch identity",
      "nav2_override":"navigation override identity (not numerical MX065 definition)",
    }
    diff_reason={
      "mapex_run":"Different bytes: prospective adds GT-v2 identity/provenance checks, New-Room paper500 hooks, defensive missing-attribute/stamp handling, and executor/runtime metadata. Ordinary stored raw/canvas/prediction arrays retain the same save/linkage and member/threshold geometry used by MX065; paper500 is inactive for Hospital MX029 and hpx_001 stored artifacts are consumed read-only.",
      "mapex_evaluator":"Only relevant diff is evaluation_canvas id derived from GT filename vs hard-coded hospital_canvas_v1; numeric canvas shape/resolution/origin, thresholds and projection are unchanged. MX065 independently binds GT-v2 and accepted hpx_001 MX018 alignment.",
      "nf_run_recorder_base":"Different bytes add paper500/New-Room CLI/executor behavior and paper500 evaluator skip. No change to ordinary Hospital recorded raw/canvas/prediction/decision linkage semantics consumed by MX065."
    }
    audit=[]
    for run in RUNS:
        meta=json.load(open(exps/run/"metadata.json")); cfg=meta.get("config_sha256",{})
        for role in list(prospective)+list(common):
            stored=cfg.get(role,"")
            if role=="runtime_mapex":
                cand=exps/run/"runtime_mapex.yaml"
            elif role in legacy and run=="hpx_001":
                cand=legacy[role]
            elif role in prospective:
                cand=prospective[role]
            else:cand=common[role]
            got=sha(cand) if cand.is_file() else ""
            match=bool(stored and got==stored)
            if stored and not match: hard.append(f"{run}:{role}:SHA_MISMATCH")
            cross_diff=run=="hpx_001" and role in diff_reason
            cls="DEFINITION_EQUIVALENT_FOR_MX065" if cross_diff and match else ("IDENTICAL_ACQUISITION_BYTES" if match else "MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE")
            reason=diff_reason.get(role,"Exact stored acquisition SHA256 matches candidate bytes; same stored identity across relevant runs.") if cls!="MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE" else "Required acquisition candidate could not be hash-reconciled."
            audit.append({
              "run_id":run,"provenance_stratum":"LEGACY_RETROSPECTIVE_GT_V2" if run=="hpx_001" else "PROSPECTIVE_GT_V2",
              "source_role":role,"metadata_config_key":role,"stored_sha256":stored,"candidate_source_path":str(cand),
              "candidate_sha256":got,"sha256_match":b(match),"comparison_stratum":"hpx_001_vs_hpx_002_005",
              "semantic_surface":surface[role],"diff_summary":"byte difference vs prospective" if cross_diff else "no relevant cross-stratum byte difference",
              "compatibility_class":cls,"compatibility_reason":reason,
              "evidence_ref":"stored metadata config_sha256 + candidate-byte SHA256"
            })
    # Checkpoint identity coverage: stored immutable artifact identities; no code-source exactness claim.
    cps={r["run_id"]:r["ensemble_checkpoints"] for r in manifest_runs}
    for member in ("G1","G2","G3"):
        vals={run:next(x for x in cps[run] if x["member"]==member) for run in RUNS}
        same=len({x["sha256"] for x in vals.values()})==1 and len({x["size_bytes"] for x in vals.values()})==1
        if not same:hard.append(f"{member}:CHECKPOINT_IDENTITY_DRIFT")
        for run,x in vals.items():
            audit.append({
              "run_id":run,"provenance_stratum":"LEGACY_RETROSPECTIVE_GT_V2" if run=="hpx_001" else "PROSPECTIVE_GT_V2",
              "source_role":f"checkpoint_{member}","metadata_config_key":"ensemble_checkpoints","stored_sha256":x["sha256"],
              "candidate_source_path":"","candidate_sha256":"","sha256_match":"",
              "comparison_stratum":"hpx_001_vs_hpx_002_005","semantic_surface":"prediction checkpoint identity",
              "diff_summary":"same stored SHA256 and byte size across all five runs" if same else "stored identity differs",
              "compatibility_class":"IDENTICAL_STORED_ARTIFACT_IDENTITY" if same else "MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE",
              "compatibility_reason":"Frozen predictions are consumed directly; checkpoint is not regenerated. Stored acquisition checkpoint SHA256+size are identical across all five runs; no claim is made that a local checkpoint candidate was acquisition-exact.",
              "evidence_ref":"metadata.ensemble_checkpoints"
            })
    material=[r for r in audit if r["compatibility_class"]=="MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE"]
    claimed=[r for r in audit if r["candidate_source_path"]]
    compat_pass=(not material and all(r["sha256_match"]==1 for r in claimed))
    if not compat_pass:hard.append("ACQUISITION_SEMANTIC_COMPATIBILITY_FAIL")

    source_manifest={
      "schema":"mx065_hospital_source_manifest_v1","task":"MX066","method_task":"MX065",
      "method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,"method_qa_commit":METHOD_QA,"pm_handoff_commit":PM_HANDOFF,
      "collection_source_revision":COLLECTION,"canonical_runs":list(RUNS),"decision_counts":COUNTS,
      "total_decisions":sum(COUNTS.values()),"gt_sha256":GT_SHA,"roi_sha256":ROI_SHA,"semantic_digest":SEM_SHA,
      "technical_abort_namespace_excluded":run_manifest.get("technical_aborts",[]),
      "runs":manifest_runs,
      "acquisition_semantic_compatibility_pass":bool(compat_pass),
      "source_freeze_hard_failures":sorted(set(hard))
    }
    wjson(out/"MX065_HOSPITAL_SOURCE_MANIFEST.json",source_manifest)
    wcsv(out/"MX065_ACQUISITION_SEMANTIC_COMPATIBILITY_AUDIT.csv",audit)
    wcsv(out/"MX065_HOSPITAL_RUN_INVENTORY.csv",inventory)
    wcsv(out/"MX065_ARTIFACT_AVAILABILITY_BY_RUN.csv",avail)
    wcsv(out/"MX065_GT_PROVENANCE_ALIGNMENT_AUDIT.csv",gta)
    summary={"source_freeze_pass":not hard,"compatibility_pass":compat_pass,"hard_failures":sorted(set(hard)),
             "runs":len(RUNS),"decisions":sum(COUNTS.values()),"compatibility_rows":len(audit),
             "candidate_byte_claim_rows":len(claimed),"candidate_byte_sha_matches":sum(r["sha256_match"]==1 for r in claimed)}
    print(json.dumps(summary,sort_keys=True))
if __name__=="__main__":main()
