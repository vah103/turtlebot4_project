"""MX071 R2 Stage 1 only: 8 sources, sealed slots and at most 80 first-goal requests."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from engine import (ART,SEAL,MODEL_ID,State,Predictor,ResourceAbort,branch_metrics,
                    execute,prediction_key,replay_key)
from native_adapter import atomic_json,frozen_gain,planning_cost,sha_array,sha_file,torch
from scoring import diagnose_snapshot,score_online

RUN=ART/"stage1"
SELECTORS=["P0","P1","P4","M0","MAPEX_ALIGNED"]


def append(path,row):
    path=Path(path)
    existing=path.read_text() if path.exists() else ""
    temp=path.with_suffix(path.suffix+".tmp")
    temp.write_text(existing+json.dumps(row,sort_keys=True,allow_nan=False)+"\n")
    temp.replace(path)


def read_rows(path):
    if not Path(path).exists(): return []
    return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]


def load_assets(map_info):
    with np.load(str(ART/(map_info["map_id"]+"_sealed.npz"))) as z:
        return z["gt"].copy(),z["component"].copy()


def execution_seal():
    path=RUN/"manifest.json"
    source_files={p.name:sha_file(p) for p in Path(__file__).parent.glob("*.py")}
    if path.exists():
        saved=json.loads(path.read_text())
        if saved["implementation_files"]!=source_files:
            raise RuntimeError("SEALED_IMPLEMENTATION_CHANGED")
        return saved
    gates={}
    for name in ["preliminary_parity","extended_parity","controller_replay_parity",
                 "cpu_probe_monitor","largest_probe_v3_monitor","scientific_contract_checks"]:
        data=json.loads((ART/(name+".json")).read_text())
        if data.get("status")!="PASS" and not data.get("all_pass",False):
            raise RuntimeError("PREFLIGHT_GATE_FAILED:"+name)
        gates[name]=sha_file(ART/(name+".json"))
    manifest=dict(SEAL,asset_seal_sha256=sha_file(ART/"manifest.json"),
                  implementation_files=source_files,gates=gates,
                  sealed_at_unix=time.time(),pipeline_version="MX071_R2_STAGE1_V1",
                  model_identity=MODEL_ID,source_trajectories=8,
                  per_map_block_wall_ceiling_s=86400,total_wall_ceiling_s=345600,
                  callback_quota="stop at next map block boundary, no substitute maps",
                  cache_limit_entries=3,stage_2="CLOSED",
                  adaptations=SEAL["adaptations"]+[
                      "LEAN_NATIVE_MAPPER_V1: drop unused constant visualization buffers",
                      "CPU_MEMBER_WORKER_V1: isolated member process, exact input, seed0; no GT",
                      "CPU_RSS_CAP_V3:5120MiB, available-memory guard256MiB",
                      "ALIGNED_REPLAN_ON_RELEASE_V1: unreachable/invalid/reached -> common PIPE decision",
                      "MAPEX_ENDPOINT_PROB_V1: native get_vis_mask(mean,prob,.8); omit overwritten binary pass",
                      "Coverage integral: right-continuous 0.1m travel/scan steps"],
                  hardware_limits=dict(worker_rss_mib=5120,minimum_available_mib=256,
                                       member_wall_s=1200,minimum_disk_mib=512),
                  optional=dict(H2="NOT_EVALUATED",H3="NOT_EVALUATED",
                                H5="DEFERRED_TRACE_ONLY",H6="DEFERRED_CLUSTERS_ONLY"))
    # Verify architecture equality; large-map G1 probe covers the frozen shared architecture.
    import yaml
    generators=[]
    for model in manifest["models"]:
        config=yaml.safe_load(Path(model["checkpoint"]).parent.parent.joinpath("config.yaml").read_text())
        generators.append(config["generator"])
    if not all(g==generators[0] for g in generators):
        raise RuntimeError("ENSEMBLE_ARCHITECTURE_NOT_EQUAL")
    manifest["ensemble_generator_config"]=generators[0]
    for model in manifest["models"]:
        if sha_file(model["checkpoint"])!=model["sha256"]:
            raise RuntimeError("CHECKPOINT_CHANGED_SINCE_ASSET_SEAL")
    for name,digest in manifest["source_files"].items():
        if sha_file(ART.parent/"pipe_source"/name)!=digest:
            raise RuntimeError("SOURCE_CHANGED_SINCE_ASSET_SEAL:"+name)
    atomic_json(path,manifest)
    return manifest


def source_run(map_info,start,manifest):
    identity=dict(map_id=map_info["map_id"],map_hash=map_info["gt_hash"],start_id=start["start_id"])
    directory=RUN/(map_info["map_id"]+"_"+start["start_id"])
    directory.mkdir(exist_ok=True)
    gt,component=load_assets(map_info)
    current=directory/"current_state.npz"
    final=directory/"final_state.npz"
    index_path=directory/"snapshot_index.json"
    if final.exists():
        restored=State.load(final,gt,component)
        summary_file=directory/"source_summary.json"
        if not summary_file.exists():
            atomic_json(summary_file,dict(identity=identity,terminal=restored.meta["terminal"],
                travel_m=restored.meta["moves"]/10.,coverage=restored.coverage(),
                decisions=restored.meta["decisions"],attempts=restored.meta["attempts"],
                recovered_after_final_write=True))
        if not any(x["identity"]==identity for x in read_rows(RUN/"source_outcomes.jsonl")):
            append(RUN/"source_outcomes.jsonl",json.loads(summary_file.read_text()))
        return restored,directory
    state=State.load(current,gt,component) if current.exists() else State(
        gt,component,start["padded_row_col"],identity)
    state.meta["execution_contract_hash"]=sha_file(RUN/"manifest.json")
    snapshots=json.loads(index_path.read_text()) if index_path.exists() else []
    predictor=Predictor(directory/"compute")
    def on_decision(s,online,prediction,timing):
        # Predetermined travel crossings only, before assigning the newly selected goal.
        pending=[t for t in [20,40,60,80] if s.meta["moves"]>=t*10 and
                 not any(t in x["target_slots_m"] for x in snapshots)]
        if not pending: return
        key=s.complete_hash()
        filename="snapshot_"+key[:20]
        s.save(directory/(filename+"_state.npz"))
        members,mean,variance=prediction
        payload=dict(predictions=members,mean=mean,variance=variance,
                     clusters=online["clusters"])
        candidates=[]
        for i,c in enumerate(online["pool"]):
            payload["path_%d"%i]=c["path"]
            payload["indices_%d"%i]=c["native_indices"]
            payload["P0_mask_%d"%i]=c["native_mask"]
            candidates.append(dict(index=i,candidate_id=c["candidate_id"],goal=c["goal"].tolist(),
                                   path_m=c["path_m"],weighted_astar_cost=c["weighted_cost"],
                                   native_sample_count=len(c["native_indices"]),P0=c["P0"]))
        np.savez_compressed(str(directory/(filename+"_predictions.npz")),**payload)
        row=dict(snapshot_id=filename,complete_state_hash=key,target_slots_m=pending,
                 observed_travel_m=s.meta["moves"]/10.,move=s.meta["moves"],
                 identity=identity,planning_contract="ALIGNED_SHARED",
                 state_file=str(directory/(filename+"_state.npz")),
                 predictions_file=str(directory/(filename+"_predictions.npz")),
                 chosen_P0=None if online["chosen"] is None else online["chosen"]["candidate_id"],
                 candidate_pool=candidates,rejected_candidates=online["rejected"],
                 full_paths_immutable=True,prediction_timing=timing,
                 P0_compute_s=online["P0_compute_s"])
        snapshots.append(row)
        atomic_json(index_path,snapshots)
        append(RUN/"snapshots.jsonl",row)
        print("SNAPSHOT",identity,pending,s.meta["moves"]/10.,flush=True)
    def progress(s):
        if s.meta["moves"]%25==0:
            s.save(current)
            atomic_json(RUN/"progress.json",dict(
                phase="SOURCE",map_id=identity["map_id"],start_id=identity["start_id"],
                travel_m=s.meta["moves"]/10.,decisions=s.meta["decisions"],
                completed_sources=len(list(RUN.glob("*/source_summary.json"))),
                completed_branch_requests=len(read_rows(RUN/"branch_outcomes.jsonl")),
                snapshot_count=len(read_rows(RUN/"snapshots.jsonl")),stage_2="CLOSED"))
        if time.time()-manifest["sealed_at_unix"]>manifest["total_wall_ceiling_s"]:
            # Complete current map block as precommitted; boundary stop is handled by main.
            pass
    execute(state,predictor,on_decision=on_decision,on_progress=progress)
    state.save(final)
    seen={t for x in snapshots for t in x["target_slots_m"]}
    for target in [20,40,60,80]:
        if target not in seen:
            append(RUN/"snapshots.jsonl",dict(identity=identity,target_slots_m=[target],
                   status="NA",reason="TERMINAL_OR_NO_NEW_GOAL_DECISION",
                   terminal=state.meta["terminal"],terminal_travel_m=state.meta["moves"]/10.))
    summary=dict(identity=identity,terminal=state.meta["terminal"],travel_m=state.meta["moves"]/10.,
                 coverage=state.coverage(),decisions=state.meta["decisions"],
                 attempts=state.meta["attempts"],snapshots=len(snapshots),
                 inference_calls=predictor.calls,member_calls=predictor.member_calls,
                 cache_hits=predictor.cache_hits,peak_worker_rss_mib=predictor.peak_rss_mib,
                 terminal_detail=state.meta.get("terminal_detail"),
                 member_timings=predictor.member_timings)
    atomic_json(directory/"source_summary.json",summary)
    append(RUN/"source_outcomes.jsonl",summary)
    return state,directory


def reconstruct_online(snapshot,observed):
    with np.load(snapshot["predictions_file"]) as z:
        members,mean,variance=z["predictions"],z["mean"],z["variance"]
        pool=[]
        for row in snapshot["candidate_pool"]:
            i=row["index"]
            pool.append(dict(row,goal=np.asarray(row["goal"]),path=z["path_%d"%i].copy(),
                             native_indices=z["indices_%d"%i].copy(),
                             native_mask=z["P0_mask_%d"%i].copy()))
    chosen=next((c for c in pool if c["candidate_id"]==snapshot["chosen_P0"]),None)
    return dict(pool=pool,rejected=snapshot["rejected_candidates"],chosen=chosen),(
        members,mean,variance)


def baseline_intervention(source,snapshot,initial_unknown):
    following=[x for x in source.decision_trace if x["move"]>snapshot["move"]]
    if following:
        event=following[0]
        seen=np.unpackbits(np.frombuffer(bytes.fromhex(event["observed_known_hex"]),dtype=np.uint8))
        mask=seen[:initial_unknown.size].reshape(initial_unknown.shape).astype(bool)
        end=event["move"]
        reason=event["release"]
    else:
        mask=source.mapper.obs_map!=.5
        end=source.meta["moves"]
        reason=source.meta.get("release_reason") or source.meta["terminal"]
    return dict(new_observed=mask & initial_unknown,end_move=end,release_reason=reason,
                poses=[x["pose"] for x in source.trace if snapshot["move"]<=x["move"]<=end])


def paired_requests(source,directory,map_info,manifest):
    index=directory/"snapshot_index.json"
    if not index.exists(): return
    gt,component=load_assets(map_info)
    existing=read_rows(RUN/"branch_outcomes.jsonl")
    completed={(x["snapshot_id"],x["selector"]) for x in existing}
    for snapshot in json.loads(index.read_text()):
        if not any(t in [20,60] for t in snapshot["target_slots_m"]): continue
        state=State.load(snapshot["state_file"],gt,component)
        online,(members,mean,variance)=reconstruct_online(snapshot,state.mapper.obs_map)
        if not online["pool"]:
            for selector in SELECTORS:
                identity=(snapshot["snapshot_id"],selector)
                if identity not in completed:
                    append(RUN/"branch_outcomes.jsonl",dict(
                        snapshot_id=identity[0],selector=selector,status="NA_EMPTY_POOL",
                        identity=snapshot["identity"],target_slots_m=snapshot["target_slots_m"]))
            continue
        diagnostic=directory/(snapshot["snapshot_id"]+"_diagnostics.json")
        choices=None
        try:
            choices=diagnose_snapshot(online,state.mapper.obs_map,np.asarray(state.meta["pose"]),
                                      members,mean,variance,gt)
            rows=[]
            masks={}
            for i,c in enumerate(online["pool"]):
                values={k:float(c[k]) for k in
                        ["P0","P1","P4","M0","M1","M4","MAPEX_ALIGNED",
                         "END_EUCLIDEAN","END_PATHCOST","PATH_EUCLIDEAN","PATH_PATHCOST"] if k in c}
                row=dict(snapshot_id=snapshot["snapshot_id"],candidate_id=c["candidate_id"],
                         goal=c["goal"].tolist(),full_path=c["path"].tolist(),
                         native_sample_indices=c["native_indices"].tolist(),
                         matched_sample_indices=c["matched_indices"].tolist(),
                         path_m=c["path_m"],weighted_astar_cost=c["weighted_astar_cost"],
                         native_denominator=max(1,len(c["native_indices"])),
                         scores=values,frozen_gains=c["frozen_gains"],
                         visibility_differences=c["visibility_differences"],
                         intrinsic_compute_s={k:c[k] for k in
                             ["P1_compute_s","P4_compute_s","matched_compute_s","MAPEX_compute_s",
                              "gt_dense_compute_s"]},
                         planning_contract="ALIGNED_SHARED",
                         score_units="frozen variance visible mass per declared denominator")
                rows.append(row)
                for name,mask in c["visibility_masks"].items():
                    masks["candidate_%d_%s"%(i,name)]=mask
            if not diagnostic.exists():
                for row in rows: append(RUN/"candidate_scores.jsonl",row)
            np.savez_compressed(str(diagnostic.with_suffix(".npz")),**masks)
            atomic_json(diagnostic,dict(status="PASS",choices={k:None if v is None else v["candidate_id"]
                                                              for k,v in choices.items()},rows=rows))
        except Exception as exc:
            atomic_json(diagnostic,dict(status="DIAGNOSTIC_ERROR",error=type(exc).__name__+": "+str(exc)))
            choices=dict(P0=online["chosen"])
        base_choice=choices.get("P0")
        base_key=replay_key(state,base_choice["goal"]) if base_choice is not None else None
        base_metrics=branch_metrics(source.trace,snapshot["move"],source.meta["terminal"])
        cache={}
        if base_key is not None:
            cache[base_key]=dict(metrics=base_metrics,
                                 intervention=baseline_intervention(source,snapshot,state.mapper.obs_map==.5),
                                 physical_id="SOURCE_SUFFIX:"+snapshot["snapshot_id"])
        for selector in SELECTORS:
            identity=(snapshot["snapshot_id"],selector)
            if identity in completed: continue
            selected=choices.get(selector)
            common=dict(snapshot_id=identity[0],selector=selector,identity=snapshot["identity"],
                        target_slots_m=snapshot["target_slots_m"],
                        current_travel_m=snapshot["observed_travel_m"],
                        complete_state_hash=snapshot["complete_state_hash"])
            if selected is None:
                append(RUN/"branch_outcomes.jsonl",dict(common,status="NOT_EVALUATED_DIAGNOSTIC_OR_INVALID"))
                continue
            key=replay_key(state,selected["goal"])
            branch_dir=directory/("branch_"+key[:20])
            branch_dir.mkdir(exist_ok=True)
            if key not in cache:
                result_path=branch_dir/"result.json"
                intervention_file=branch_dir/"intervention.npz"
                if result_path.exists():
                    cached=json.loads(result_path.read_text())
                    with np.load(str(intervention_file)) as z:
                        intervention=dict(new_observed=z["new_observed"],
                                          end_move=int(z["end_move"]),
                                          release_reason=str(z["release_reason"]),
                                          poses=json.loads(str(z["poses"])))
                    cache[key]=dict(metrics=cached["metrics"],intervention=intervention,
                                    physical_id=cached["physical_id"])
                else:
                    branch_state=State.load(snapshot["state_file"],gt,component)
                    current=branch_dir/"current_state.npz"
                    # Rerun a physically interrupted branch from exact snapshot; <=1 same-config retry.
                    predictor=Predictor(branch_dir/"compute")
                    def branch_progress(s):
                        if s.meta["moves"]%25==0:
                            s.save(current)
                            atomic_json(RUN/"progress.json",dict(
                                phase="BRANCH",selector=selector,map_id=map_info["map_id"],
                                start_id=snapshot["identity"]["start_id"],travel_m=s.meta["moves"]/10.,
                                remaining_m=(1000-s.meta["moves"])/10.,stage_2="CLOSED"))
                    intervention=execute(branch_state,predictor,on_progress=branch_progress,
                                         first_goal=selected["goal"])
                    branch_state.save(branch_dir/"final_state.npz")
                    if intervention is None: raise RuntimeError("MISSING_ACTUAL_INTERVENTION")
                    np.savez_compressed(str(intervention_file),
                                        new_observed=intervention["new_observed"],
                                        end_move=intervention["end_move"],
                                        release_reason=intervention["release_reason"],
                                        poses=json.dumps(intervention["poses"]))
                    metrics=branch_metrics(branch_state.trace,snapshot["move"],branch_state.meta["terminal"])
                    physical_id="BRANCH:"+key
                    atomic_json(result_path,dict(metrics=metrics,physical_id=physical_id,
                                                 member_calls=predictor.member_calls,
                                                 inference_calls=predictor.calls,cache_hits=predictor.cache_hits))
                    cache[key]=dict(metrics=metrics,intervention=intervention,physical_id=physical_id)
            result=cache[key]
            metric=dict(result["metrics"])
            delta=None if metric["Q"] is None or base_metrics["Q"] is None else metric["Q"]-base_metrics["Q"]
            intervention=result["intervention"]
            observed_gain=frozen_gain(variance,state.mapper.obs_map==.5,intervention["new_observed"])
            row=dict(common,status="EVALUATED",goal=selected["goal"].tolist(),
                     reuse_key=key,physical_id=result["physical_id"],metrics=metric,
                     baseline_metrics=base_metrics,delta_Q_vs_P0=delta,
                     execution_equivalent_to_P0=key==base_key,structural_zero=key==base_key,
                     selector_intrinsic_compute_s=(
                         snapshot["P0_compute_s"] if selector=="P0" else
                         sum(c.get({"P1":"P1_compute_s","P4":"P4_compute_s","M0":"matched_compute_s",
                                    "MAPEX_ALIGNED":"MAPEX_compute_s"}[selector],0.)
                             for c in online["pool"])),
                     executed_sensor_frozen_gain=observed_gain,
                     intervention_end_move=intervention["end_move"],
                     intervention_release=intervention["release_reason"],
                     intervention_actual_poses=intervention["poses"],
                     declared_sensor_horizon="FULL_NATIVE_SAMPLED_ROUTE",
                     actual_sensor_horizon="FIRST_GOAL_UNTIL_RELEASE_OR_TERMINAL",
                     same_pose_schedule_claim=False,
                     continuation="PIPE_ALIGNED",residual_label="EXECUTION_SENSOR_CONTRACT_GAP",
                     H7_eligible=(1000-snapshot["move"]>=200 and
                                  len({tuple(c["goal"]) for c in choices.values() if c is not None})>=2),
                     uncertainty_posterior_entropy_reduction="NOT_CLAIMED")
            append(RUN/"branch_outcomes.jsonl",row)
            completed.add(identity)
            print("BRANCH_DONE",selector,map_info["map_id"],snapshot["observed_travel_m"],
                  metric["Q"],"alias" if key==base_key else "distinct",flush=True)


def summarize(status):
    rows=read_rows(RUN/"branch_outcomes.jsonl")
    sources=read_rows(RUN/"source_outcomes.jsonl")
    usable=[x for x in rows if x.get("status")=="EVALUATED"]
    fields=["map_id","start_id","direction","paired_slots","non_equivalent_slots","mean_delta_Q",
            "behavior_failure_difference"]
    summary=[]
    screens=[]
    for selector in ["P1","P4"]:
        focal=[x for x in usable if x["selector"]==selector and x["delta_Q_vs_P0"] is not None]
        grouped={}
        for row in focal:
            identity=(row["identity"]["map_id"],row["identity"]["start_id"])
            grouped.setdefault(identity,[]).append(row)
        for (map_id,start_id),items in grouped.items():
            summary.append(dict(map_id=map_id,start_id=start_id,direction=selector,
                                paired_slots=len(items),
                                non_equivalent_slots=sum(not x["execution_equivalent_to_P0"] for x in items),
                                mean_delta_Q=float(np.mean([x["delta_Q_vs_P0"] for x in items])),
                                behavior_failure_difference=sum(
                                    int(x["metrics"]["behavior_failure"])-int(x["baseline_metrics"]["behavior_failure"])
                                    for x in items)))
        groups=[x for x in summary if x["direction"]==selector]
        macro={}
        for row in groups: macro.setdefault(row["map_id"],[]).append(row["mean_delta_Q"])
        map_means={k:float(np.mean(v)) for k,v in macro.items()}
        overall=float(np.mean(list(map_means.values()))) if map_means else None
        supported=[x for x in groups if x["non_equivalent_slots"]>0]
        full_maps=len(map_means)==4
        positive=(full_maps and overall is not None and overall>=.01 and
                  sum(v>0 for v in map_means.values())>=3 and len(supported)>=6 and
                  len({x["map_id"] for x in supported})>=3 and
                  all(x["behavior_failure_difference"]<=0 for x in groups))
        verdict="PIPE_RELEVANT_SIGNAL_PIPE_ALIGNED" if positive else "INCONCLUSIVE_OR_NO_QUALIFYING_SIGNAL"
        if full_maps and len(groups)>=6 and not any(x["non_equivalent_slots"] for x in groups):
            verdict="NO_ACTION_EFFECT_IN_PILOT"
        screens.append(dict(direction=selector,verdict=verdict,map_macro_delta_Q=overall,
                            map_means=map_means,unique_evaluable_map_start_units=len(groups),
                            unique_non_equivalent_map_start_units=len(supported)))
    # H7 uses only distinct already executed/reused actions and a common remaining budget.
    points={}
    for row in usable:
        if row.get("H7_eligible") and row["metrics"]["Q"] is not None:
            points.setdefault(row["snapshot_id"],{})[row["reuse_key"]]=row
    h7=[]
    for sid,distinct in points.items():
        if len(distinct)<2: continue
        actions=list(distinct.values())
        prefix_best=max(actions,key=lambda x:x["metrics"]["prefix_gain_10m"])
        horizon_best=max(actions,key=lambda x:x["metrics"]["Q"])
        prefix_max=prefix_best["metrics"]["prefix_gain_10m"]
        prefix_ties=sum(x["metrics"]["prefix_gain_10m"]==prefix_max for x in actions)
        h7.append(dict(snapshot_id=sid,identity=prefix_best["identity"],
                       distinct_tested_actions=len(actions),
                       tested_prefix_regret=horizon_best["metrics"]["Q"]-prefix_best["metrics"]["Q"],
                       prefix_tie_count=prefix_ties,
                       prefix_winner=prefix_best["reuse_key"],remaining_horizon_winner=horizon_best["reuse_key"],
                       failure_difference=int(horizon_best["metrics"]["behavior_failure"])-
                                          int(prefix_best["metrics"]["behavior_failure"])))
    atomic_json(RUN/"h7_tested_action_summary.json",h7)
    h7_groups={}
    for row in h7:
        key=(row["identity"]["map_id"],row["identity"]["start_id"])
        h7_groups.setdefault(key,[]).append(row)
    h7_maps={}
    for (map_id,start_id),items in h7_groups.items():
        effect=float(np.mean([x["tested_prefix_regret"] for x in items]))
        h7_maps.setdefault(map_id,[]).append(effect)
        summary.append(dict(map_id=map_id,start_id=start_id,direction="H7",
                            paired_slots=len(items),non_equivalent_slots=len(items),
                            mean_delta_Q=effect,
                            behavior_failure_difference=sum(x["failure_difference"] for x in items)))
    h7_means={k:float(np.mean(v)) for k,v in h7_maps.items()}
    h7_macro=float(np.mean(list(h7_means.values()))) if h7_means else None
    h7_positive=(len(h7_means)==4 and len(h7_groups)>=6 and len(h7_maps)>=3 and
                 h7_macro is not None and h7_macro>=.01 and
                 sum(v>0 for v in h7_means.values())>=3 and
                 all(x["failure_difference"]<=0 for x in h7))
    screens.append(dict(direction="H7",
                        verdict="BOUNDED_TESTED_HORIZON_SIGNAL" if h7_positive else
                                "INCONCLUSIVE_OR_NO_QUALIFYING_SIGNAL",
                        map_macro_delta_Q=h7_macro,map_means=h7_means,
                        unique_evaluable_map_start_units=len(h7_groups),
                        unique_non_equivalent_map_start_units=len(h7_groups)))
    with (RUN/"map_start_summary.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary)
    report=dict(status=status,completed_sources=len(sources),logical_branch_requests=len(rows),
                unique_physical_execution_ids=len({x["physical_id"] for x in usable}),
                snapshot_slots=sum(len(x["target_slots_m"]) for x in read_rows(RUN/"snapshots.jsonl")),screens=screens,
                cohort="MAP_ID_ONLY / TRAIN_OVERLAP_UNVERIFIED",stage_2="CLOSED",
                H7="BOUNDED_TESTED_ACTION_COMPARISONS_ONLY",
                H4="ORACLE_HEADROOM_NOT_ONLINE_RECOVERABLE_METHOD")
    atomic_json(RUN/"screening_summary.json",report)
    text="# MX071 R2 — DELL Stage 1\n\nStatus: "+status+"\n\n"
    text+="Completed sources: %d/8; branch requests: %d (ceiling 80).\n\n"%(len(sources),len(rows))
    for screen in screens:
        text+="- %s: %s; map-macro delta Q=%s; evaluable map/start=%d, non-equivalent=%d.\n"%(
            screen["direction"],screen["verdict"],screen["map_macro_delta_Q"],
            screen["unique_evaluable_map_start_units"],screen["unique_non_equivalent_map_start_units"])
    text+="\nNo CODE_REFERENCE utility or held-out-building claim. H4 is oracle headroom. "
    text+="Stage 2 closed; H2/H3 not evaluated; H5/H6 deferred. "
    text+="Execution/scan residuals retain EXECUTION_SENSOR_CONTRACT_GAP.\n"
    (RUN/"direction_screening.md").write_text(text)
    return report


def main():
    RUN.mkdir(exist_ok=True)
    np.random.seed(0)
    torch.manual_seed(0)
    manifest=execution_seal()
    if not (RUN/"error_update_diagnostics.jsonl").exists():
        append(RUN/"error_update_diagnostics.jsonl",dict(
            H2="NOT_EVALUATED",H3="NOT_EVALUATED",
            reason="Optional processing omitted; no extra rollouts or all-train dependency"))
    for map_info in manifest["maps"]:
        if time.time()-manifest["sealed_at_unix"]>manifest["total_wall_ceiling_s"]:
            summarize("PARTIAL_SCREEN_RESOURCE_QUOTA")
            return
        block_started=time.time()
        pairs=[]
        for start in map_info["starts"]:
            state,directory=source_run(map_info,start,manifest)
            pairs.append((state,directory))
        for state,directory in pairs:
            paired_requests(state,directory,map_info,manifest)
        summary=summarize("RUNNING")
        print("MAP_BLOCK_DONE",map_info["map_id"],json.dumps(summary),flush=True)
        if time.time()-block_started>manifest["per_map_block_wall_ceiling_s"]:
            summarize("PARTIAL_SCREEN_MAP_BLOCK_QUOTA")
            return
    issues=any(x["terminal"] in ["RESOURCE_ABORT","INFERENCE_ERROR","PLANNER_ERROR"]
               for x in read_rows(RUN/"source_outcomes.jsonl"))
    summarize("STAGE1_ATTEMPTED_WITH_INFRASTRUCTURE_GAPS" if issues else "STAGE1_COMPLETE")
    atomic_json(RUN/"progress.json",dict(phase="COMPLETE",stage_2="CLOSED",
                                       completed_sources=8,screening_file=str(RUN/"direction_screening.md")))


if __name__=="__main__":
    main()
