#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import mx048_generator as gen
from mx048_acquisition import (
    COMPONENT_TAGS, COMPONENT_STATUSES, GENERATOR_ID,
    build_manifest, consume_unseal_once, derive_seed32, find_layout,
    metadata_for_binary, metadata_for_csv, require_unsealed, reserve_decision,
    resolve_run, run_provenance_from_args, sha256_file,
    should_run_offline_evaluator, verify_layout_identity, write_canonical_json,
)

RESULT_DIR_REL = "mapex_lab/analysis/mx048_preflight_results"
MANIFEST_REL = "MX045_ACQUISITION_MANIFEST.json"
TOKEN = "MX048_PREFLIGHT_PASS_READY_FOR_MX046_COLLECTION"
IDENTITY_TEMPLATES = (
    gen.GENERATED_WORLD_REL, gen.GEOMETRY_REL, gen.LAYOUT_CONFIG_REL,
    gen.GEN_AUDIT_REL, gen.GT_REL, gen.ROI_REL, gen.GT_SUMMARY_REL,
    gen.GT_BINDING_REL,
)

def run(cmd, cwd=None, env=None, check=True, timeout=None):
    return subprocess.run(
        cmd, cwd=cwd, env=env, check=check, timeout=timeout,
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )

def git(root: Path, *args: str) -> str:
    return run(["git", "-C", str(root), *args]).stdout.strip()

def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)

def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))

def deterministic_hashes(root: Path, seed: int) -> dict[str, str]:
    return {
        template.format(seed=seed): sha256_file(root / template.format(seed=seed))
        for template in IDENTITY_TEMPLATES
    }

def semantic_summary(root: Path, seed: int) -> dict:
    audit = load_json(root / gen.GEN_AUDIT_REL.format(seed=seed))
    binding = load_json(root / gen.GT_BINDING_REL.format(seed=seed))
    return {
        "accepted_attempt": audit["accepted_attempt"],
        "geometry_parameter_digest": audit["geometry_parameter_digest"],
        "world_identity_sha256": binding["world_identity_sha256"],
        "gt_semantic_digest": binding["semantic_digest"],
        "attempts": audit["attempts"],
        "v6": audit["v6"],
        "v7_zone_area_m2": audit["v7_zone_area_m2"],
    }

def worktree_generate(repo_root: Path, label: str):
    parent = Path(tempfile.mkdtemp(prefix=f"mx048_{label}_"))
    wt = parent / "repo"
    run(["git", "-C", str(repo_root), "worktree", "add", "--detach", str(wt), "HEAD"])
    env = os.environ.copy()
    env["PYTHONPATH"] = str(wt / "mapex_lab/scripts")
    proc = run(
        [sys.executable, str(wt / "mapex_lab/scripts/mx048_generator.py"),
         "--repo-root", str(wt)],
        env=env, check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"{label} generator failed:\n{proc.stdout}")
    return wt, json.loads(proc.stdout), parent

def cleanup_worktree(repo_root: Path, wt: Path, parent: Path) -> None:
    run(["git", "-C", str(repo_root), "worktree", "remove", "--force", str(wt)], check=False)
    shutil.rmtree(parent, ignore_errors=True)

def check_p1(repo_root: Path, result_dir: Path, cross_machine_json: Path | None):
    wt_a = wt_b = par_a = par_b = None
    rows = []
    try:
        wt_a, a, par_a = worktree_generate(repo_root, "root_A")
        wt_b, b, par_b = worktree_generate(repo_root, "root_B")
        by_a = {int(x["layout_seed"]): x for x in a}
        by_b = {int(x["layout_seed"]): x for x in b}
        for seed in gen.LAYOUT_SEEDS:
            ha, hb = deterministic_hashes(wt_a, seed), deterministic_hashes(wt_b, seed)
            sa, sb = semantic_summary(wt_a, seed), semantic_summary(wt_b, seed)
            roots_absent = True
            for rel in ha:
                for root, data in (
                    (wt_a, (wt_a / rel).read_bytes()),
                    (wt_b, (wt_b / rel).read_bytes()),
                ):
                    if str(wt_a).encode() in data or str(wt_b).encode() in data:
                        roots_absent = False
            ok = (
                by_a[seed]["status"] == "ACCEPTED"
                and by_b[seed]["status"] == "ACCEPTED"
                and ha == hb and sa == sb and roots_absent
            )
            rows.append({
                "layout_seed": seed,
                "byte_identity_equal": ha == hb,
                "semantic_identity_equal": sa == sb,
                "absolute_root_absent": roots_absent,
                "status": "PASS" if ok else "FAIL",
            })
        local_pass = all(x["status"] == "PASS" for x in rows)
        cross_pass = False
        cross_detail = "CROSS_MACHINE_CHECK_NOT_EXECUTED"
        if cross_machine_json and cross_machine_json.is_file():
            cm = load_json(cross_machine_json)
            seed = int(cm["layout_seed"])
            if seed == 45001:
                cross_pass = (
                    cm["deterministic_hashes"] == deterministic_hashes(wt_a, seed)
                    and cm["semantic_summary"] == semantic_summary(wt_a, seed)
                )
                cross_detail = "PASS" if cross_pass else "HASH_OR_SEMANTIC_MISMATCH"
            else:
                cross_detail = f"unexpected_seed:{seed}"
        write_csv(result_dir / "MX048_IDENTITY_LOCATION_INDEPENDENCE_AUDIT.csv", rows)
        return local_pass and cross_pass, cross_detail
    finally:
        if wt_a is not None:
            cleanup_worktree(repo_root, wt_a, par_a)
        if wt_b is not None:
            cleanup_worktree(repo_root, wt_b, par_b)

def final_generation(repo_root: Path, result_dir: Path):
    results = gen.generate_set(repo_root)
    write_csv(
        result_dir / "MX048_LAYOUT_GENERATION_AUDIT.csv",
        [{k:v for k,v in row.items() if k != "attempts"} for row in results],
    )
    return results

def generator_provenance(repo_root: Path) -> dict:
    return {
        "generator_id": gen.GENERATOR_ID,
        "technical_repo_commit": git(repo_root, "rev-parse", "HEAD"),
        "generator_blob": git(repo_root, "hash-object", str(repo_root/"mapex_lab/scripts/mx048_generator.py")),
        "acquisition_guard_blob": git(repo_root, "hash-object", str(repo_root/"mapex_lab/scripts/mx048_acquisition.py")),
        "accepted_gt_generator_blob": git(repo_root, "hash-object", str(repo_root/"mapex_lab/scripts/generate_new_room_ground_truth.py")),
        "base_world_sha256": sha256_file(repo_root/gen.BASE_WORLD_REL),
        "base_world_reference_blob": "bbafb79379fc665d6f3f8c6d00bb2d2e6b04db43",
        "mx045_method_commit": gen.MX045_METHOD_COMMIT,
        "mx045_method_blob": gen.MX045_METHOD_BLOB,
        "mx048_contract_commit": gen.MX048_CONTRACT_COMMIT,
        "mx048_contract_blob": gen.MX048_CONTRACT_BLOB,
        "npy_version": list(gen.NPY_VERSION),
        "npz_compression": "ZIP_STORED",
        "zip_timestamp": list(gen.ZIP_TIMESTAMP),
        "obstacle_vertical_rendering": {"center_z_m":0.6,"height_m":1.2},
    }

def check_p2(results):
    digests = [r.get("geometry_parameter_digest") for r in results if r["status"]=="ACCEPTED"]
    return len(digests)==12 and len(set(digests))==12, f"{len(set(digests))}/{len(digests)} unique"

def check_p3(repo_root, results):
    for r in results:
        if r["status"] != "ACCEPTED":
            return False, f"seed {r['layout_seed']} status={r['status']}"
        audit = load_json(repo_root/r["audit_path"])
        expected = {f"V{i}":"PASS" for i in range(1,10)}
        if audit["attempts"][-1].get("validation_results") != expected:
            return False, f"seed {r['layout_seed']} lacks V1-V9 PASS"
    return True, "12/12 accepted; V1-V9 PASS"

def check_p4(manifest):
    expected = {}
    for s in gen.PRIMARY_DEVELOPMENT: expected[s]=("development","primary")
    for s in gen.PRIMARY_CONFIRMATION: expected[s]=("confirmation","primary")
    expected[45011]=("development","reserve")
    expected[45012]=("confirmation","reserve")
    for x in manifest["layouts"]:
        if expected[int(x["layout_seed"])] != (x["split"],x["primary_or_reserve"]):
            return False, f"mapping mismatch {x['layout_seed']}"
    return True, "exact split/role mapping"

def check_p5(manifest):
    for layout in manifest["layouts"]:
        if layout["generator_attempt_status"]!="ACCEPTED":
            continue
        r1,r2 = layout["runs"]
        for tag in COMPONENT_TAGS:
            key=f"{tag}_seed"
            if r1[key]!=derive_seed32(layout["layout_seed"],1,tag):
                return False,"derivation mismatch"
            if r2[key]!=derive_seed32(layout["layout_seed"],2,tag):
                return False,"derivation mismatch"
            if r1[key]==r2[key]:
                return False,"run seeds collide"
        if r1["expected_world_identity_sha256"]!=r2["expected_world_identity_sha256"]:
            return False,"world differs by run seed"
        if r1["expected_layout_config_sha256"]!=r2["expected_layout_config_sha256"]:
            return False,"config differs by run seed"
    return True,"24 run slots recompute; geometry/config unchanged by run seed"

def gz_shell(command, timeout=20):
    return run(["bash","-lc",f"source /opt/ros/jazzy/setup.bash && {command}"],check=False,timeout=timeout)

def q(value):
    import shlex
    return shlex.quote(str(value))

def check_p6(result_dir, manifest):
    help_out = gz_shell("gz sim -h",10).stdout
    use_trace = "Pass a custom seed value to the random" in help_out and "number generator" in help_out
    probe = result_dir/"mx048_seed_probe.sdf"
    probe.write_text(
        '<?xml version="1.0"?>\n<sdf version="1.9"><world name="seed_probe">'
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
        '<physics name="default" type="ignored"><max_step_size>0.001</max_step_size></physics>'
        '</world></sdf>\n', encoding="utf-8"
    )
    rows, logs = [], []
    all_bound = True
    for layout in manifest["layouts"]:
        for rr in layout["runs"]:
            for tag in COMPONENT_TAGS:
                seed=int(rr[f"{tag}_seed"])
                if tag=="gazebo":
                    proc=gz_shell(f"timeout 8 gz sim -v 4 -r -s --iterations 1 --seed {seed} {q(probe)}",12)
                    bound=("Setting seed value:" in proc.stdout and str(seed) in proc.stdout)
                    all_bound &= bound
                    status="SEEDED_EFFECTIVE" if bound and use_trace else "SEED_INTERFACE_PRESENT_NOT_VERIFIED"
                    logs.append(f"layout={layout['layout_seed']} run={rr['run_seed']} seed={seed}\n{proc.stdout}\n")
                    binding="MX048_GAZEBO_SEED_BINDING_PROBE.log"
                    use="installed gz sim -h states --seed passes custom seed to random number generator"
                    reason=""
                else:
                    status="UNSEEDED_RUNTIME_NONDETERMINISM"
                    binding=use=""
                    reason="no verified seed interface; derived seed retained in provenance"
                rows.append({
                    "layout_seed":layout["layout_seed"],"run_seed":rr["run_seed"],
                    "component_tag":tag,"derived_seed":seed,"status":status,
                    "component_version":"Gazebo Sim 8.11.0" if tag=="gazebo" else "frozen_runtime_component",
                    "binding_proof":binding,"use_proof":use,"reason":reason,
                })
    (result_dir/"MX048_GAZEBO_SEED_BINDING_PROBE.log").write_text("\n".join(logs),encoding="utf-8")
    write_csv(result_dir/"MX048_RUN_SEED_AUDIT.csv",rows)
    effective=any(x["status"]=="SEEDED_EFFECTIVE" for x in rows)
    enums=all(x["status"] in COMPONENT_STATUSES for x in rows)
    return effective and enums and all_bound and use_trace, (
        "Gazebo SEEDED_EFFECTIVE via startup readback + installed RNG seed trace"
        if effective and use_trace else "RUN_SEED_NO_EFFECTIVE_CHANNEL"
    )

def check_p7(repo_root):
    source=(repo_root/"mapex_lab/scripts/mapex_run.py").read_text()
    return (
        not should_run_offline_evaluator(True)
        and "should_run_offline_evaluator(args.acquisition_only)" in source
    ), "acquisition_only suppresses offline evaluator"

def sealing_tests(manifest_sha):
    fixture=Path(tempfile.mkdtemp(prefix="mx048_seal_"))
    try:
        csvp=fixture/"coverage.csv"
        csvp.write_text("time_s,coverage,known_area,delta_known_area\n1,0.5,10,1\n2,0.6,11,1\n")
        occ=fixture/"occupancy.npy"; np.save(occ,np.asarray([[0,100],[-1,0]],dtype=np.int16),allow_pickle=False)
        meta=metadata_for_csv(csvp)
        metadata_ok=meta["row_count"]==2 and "0.5" not in json.dumps(meta)
        denied={}
        for kind in ("coverage","known_area","delta_known_area","occupancy_map","target","evaluator","gt","roi"):
            try:
                require_unsealed(kind,None); denied[kind]=False
            except PermissionError as e:
                denied[kind]=str(e)=="CONFIRMATION_NUMERIC_CONTENT_SEALED"
        hash_allowed=metadata_for_binary(occ,"npy")["sha256"]==sha256_file(occ)
        p8=metadata_ok and all(denied.values()) and hash_allowed

        freeze=fixture/"MX045_DEVELOPMENT_FREEZE.json"; freeze.write_text('{"frozen":true}\n')
        record={
            "contract_id":GENERATOR_ID,
            "mx045_method_commit":gen.MX045_METHOD_COMMIT,"mx045_method_blob":gen.MX045_METHOD_BLOB,
            "mx048_contract_commit":gen.MX048_CONTRACT_COMMIT,"mx048_contract_blob":gen.MX048_CONTRACT_BLOB,
            "mx046_collection_integrity_qa_accept_reference":"SYNTHETIC_QA_ACCEPT",
            "development_freeze_path":"MX045_DEVELOPMENT_FREEZE.json",
            "development_freeze_sha256":sha256_file(freeze),
            "user_pm_authorization_reference":"SYNTHETIC_AUTH",
            "confirmation_run_ids":["synthetic"],
            "acquisition_manifest_sha256":manifest_sha,
            "confirmation_gt_binding_hashes":["synthetic"],
            "unseal_actor":"synthetic","utc_timestamp":"2000-01-01T00:00:00Z","unseal_once":True,
        }
        unseal=fixture/"MX045_CONFIRMATION_UNSEAL.json"; write_canonical_json(unseal,record)
        marker=fixture/"consumed.json"
        consume_unseal_once(unseal,marker,{"acquisition_manifest_sha256":manifest_sha})
        positive=True
        try: require_unsealed("coverage",unseal,{"acquisition_manifest_sha256":manifest_sha})
        except PermissionError: positive=False
        missing=dict(record); missing["development_freeze_sha256"]=""
        mp=fixture/"missing.json"; write_canonical_json(mp,missing)
        missing_denied=False
        try: consume_unseal_once(mp,fixture/"m2.json",{})
        except PermissionError: missing_denied=True
        inconsistent=dict(record); inconsistent["acquisition_manifest_sha256"]="0"*64
        ip=fixture/"inconsistent.json"; write_canonical_json(ip,inconsistent)
        inconsistent_denied=False
        try: consume_unseal_once(ip,marker,{})
        except PermissionError: inconsistent_denied=True
        p9=positive and missing_denied and inconsistent_denied
        return p8,p9,{
            "metadata_ok":metadata_ok,"denied":denied,"hash_allowed":hash_allowed,
            "positive_unseal":positive,"missing_freeze_denied":missing_denied,
            "second_inconsistent_denied":inconsistent_denied,
        }
    finally:
        shutil.rmtree(fixture,ignore_errors=True)

def check_p10(repo_root,manifest_path):
    resolved=resolve_run(repo_root,manifest_path,45001,1)
    proc=run([
        str(repo_root/".run_core"),"--method","mapex","--environment","new_room",
        "--record","yes","--mx045-manifest",str(manifest_path),
        "--layout-seed","45001","--run-seed","1","--acquisition-only","--mx045-resolve-only",
    ],cwd=repo_root,check=False,timeout=30)
    env=os.environ.copy()
    env["TURTLEBOT4_PROJECT_ROOT"]=str(repo_root)
    dry=run([
        "ros2","launch","frontier_exploration","hospital_flat_simulation.launch.py",
        f"world:={resolved['world_path']}",
        f"gz_seed:={resolved['gazebo_seed']}",
        f"x_pose:={resolved['spawn_x_m']}",
        f"y_pose:={resolved['spawn_y_m']}",
        f"yaw:={resolved['spawn_yaw_rad']}",
        f"mx048_world_sha256:={resolved['world_sha256']}",
        f"mx048_layout_config:={resolved['layout_config_path']}",
        f"mx048_layout_config_sha256:={resolved['layout_config_sha256']}",
        "mx048_dry_launch:=true","use_rviz:=False","headless:=True",
    ],cwd=repo_root,env=env,check=False,timeout=30)
    a=(repo_root/"mapex_lab/launch/slam.launch.py").read_text()
    c=(repo_root/"ros2_ws/src/frontier_exploration/launch/tb4_simulation_safe.launch.py").read_text()
    expected_spawn=f"spawn=({resolved['spawn_x_m']},{resolved['spawn_y_m']},{resolved['spawn_yaw_rad']})"
    ok=(proc.returncode==0 and "MX048_RESOLVE_ONLY_PASS" in proc.stdout
        and dry.returncode==0 and "MX048_LAUNCH_BINDING_PASS" in dry.stdout
        and resolved["world_sha256"] in dry.stdout
        and resolved["layout_config_sha256"] in dry.stdout
        and str(Path(resolved["layout_config_path"]).resolve()) in dry.stdout
        and expected_spawn in dry.stdout
        and "'mx048_world_sha256': mx048_world_sha256" in a
        and "'mx048_layout_config': mx048_layout_config" in a
        and "'--seed', gz_seed" in c)
    return ok,(proc.stdout+"\n--- DRY LAUNCH ---\n"+dry.stdout)[-6000:]

def check_p11(repo_root,manifest_path):
    r=resolve_run(repo_root,manifest_path,45001,1)
    ns=SimpleNamespace(
        run_id=r["run_id"],layout_seed=45001,run_seed=1,split=r["split"],
        primary_or_reserve=r["primary_or_reserve"],world_identity_sha256=r["world_identity_sha256"],
        actual_world=r["world_path"],actual_world_rel=r["world_rel"],
        layout_config=r["layout_config_path"],layout_config_rel=r["layout_config_rel"],
        gt_binding=r["gt_binding_path"],gt_binding_rel=r["gt_binding_rel"],
        gazebo_seed=r["gazebo_seed"],exploration_seed=r["exploration_seed"],
        planner_seed=r["planner_seed"],sensor_seed=r["sensor_seed"],
        spawn_x_m=r["spawn_x_m"],spawn_y_m=r["spawn_y_m"],spawn_yaw_rad=r["spawn_yaw_rad"],
        acquisition_only=True,confirmation_state=r["confirmation_state"],
        manifest_sha256=r["manifest_sha256"],
    )
    p=run_provenance_from_args(ns)
    l=find_layout(load_json(manifest_path),45001)
    ok=(p["environment_world"]==l["world_path"]
        and p["actual_world_sha256"]==l["world_sha256"]
        and p["layout_config_sha256"]==l["layout_config_sha256"]
        and p["world_identity_sha256"]==l["world_identity_sha256"]
        and p["launch_spawn"]=={"x_m":0.0,"y_m":3.0,"yaw_rad":0.0}
        and p["acquisition_only"] is True)
    return ok,json.dumps({k:p[k] for k in ("environment_world","actual_world_sha256","layout_config_sha256","world_identity_sha256","launch_spawn","acquisition_only")},sort_keys=True)

def check_p12(repo_root,manifest_path):
    manifest=load_json(manifest_path); l=find_layout(manifest,45001)
    tmp=Path(tempfile.mkdtemp(prefix="mx048_mut_"))
    try:
        for k in ("world_path","layout_config_path","gt_binding_path"):
            dst=tmp/l[k]; dst.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(repo_root/l[k],dst)
        b=load_json(repo_root/l["gt_binding_path"])
        for k in ("gt_path","roi_path"):
            dst=tmp/b[k]; dst.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(repo_root/b[k],dst)
        mp=tmp/"manifest.json"; shutil.copy2(manifest_path,mp)
        wp=tmp/l["world_path"]; data=bytearray(wp.read_bytes()); data[-2]^=1; wp.write_bytes(data)
        denied=False
        try: resolve_run(tmp,mp,45001,1)
        except RuntimeError as e: denied=str(e)=="MX048_WORLD_MUTATION_DETECTED"
        return denied,"one-byte world mutation rejected"
    finally:
        shutil.rmtree(tmp,ignore_errors=True)

def check_p13(repo_root,manifest):
    try:
        for l in manifest["layouts"]: verify_layout_identity(repo_root,l)
        return True,"12/12 bindings verify"
    except Exception as e: return False,str(e)

def check_p14(repo_root,manifest_path):
    for seed in gen.LAYOUT_SEEDS:
        r=resolve_run(repo_root,manifest_path,seed,1)
        if r["world_rel"]==gen.BASE_WORLD_REL or r["world_rel"].endswith("/new_room.sdf"):
            return False,f"fixed fallback {seed}"
    return True,"generated world path only"

def check_p15(repo_root,seal):
    forbidden=("MX045_TARGETS_PER_DECISION.csv","MX045_DEVELOPMENT_MODELS.csv","MX045_CONFIRMATION_METRICS.csv")
    return all(not (repo_root/x).exists() for x in forbidden) and all(seal["denied"].values()),"no derived outputs; numeric denylist enforced"

def check_p16():
    a=reserve_decision("development",[{"layout_seed":45001,"primary_or_reserve":"primary","technical_invalid":True}])
    b=reserve_decision("development",[{"layout_seed":45001,"primary_or_reserve":"primary","technical_invalid":False,"scientific_weakness":True}])
    c=reserve_decision("development",[
        {"layout_seed":45001,"primary_or_reserve":"primary","technical_invalid":True},
        {"layout_seed":45002,"primary_or_reserve":"primary","technical_invalid":True},
    ])
    ok=a["action"]=="ACTIVATE_WHOLE_LAYOUT_RESERVE" and b["action"]=="NO_RESERVE" and c["action"]=="GLOBAL_INSUFFICIENT_BLOCK"
    return ok,json.dumps({"technical":a,"scientific":b,"second_failure":c},sort_keys=True)

def check_p17(repo_root):
    source=(repo_root/"mapex_lab/scripts/mx048_generator.py").read_text()
    tree=ast.parse(source)
    imported=[]
    for n in ast.walk(tree):
        if isinstance(n,ast.Import): imported.extend(a.name for a in n.names)
        elif isinstance(n,ast.ImportFrom) and n.module: imported.append(n.module)
    bad=any(x.startswith(("mapex_lab.experiments","evaluate_mapex_run","mapex")) for x in imported)
    bad=bad or any(x in source for x in ("mapex_lab/experiments/","MX045_TARGETS_PER_DECISION","Oracle"))
    return not bad,"generator imports contain no telemetry/scoring/outcome dependency"

def check_p18(repo_root,manifest,manifest_path):
    top={"schema_version","mx045_method_commit","mx045_method_blob","mx048_contract_commit","mx048_contract_blob","generator_id","generator_commit","generator_blob","technical_repo_commit","created_utc","manifest_state","layouts"}
    lr={"layout_seed","split","primary_or_reserve","planned_status","generator_attempt_status","accepted_attempt","geometry_parameter_digest","world_path","world_sha256","layout_config_path","layout_config_sha256","world_identity_sha256","gt_binding_path","gt_sha256","roi_sha256","runs"}
    rr={"run_id","run_seed","gazebo_seed","exploration_seed","planner_seed","sensor_seed","expected_world_identity_sha256","expected_layout_config_sha256"}
    if not top.issubset(manifest): return False,"missing top keys"
    for l in manifest["layouts"]:
        if not lr.issubset(l): return False,"missing layout keys"
        if any(not rr.issubset(r) for r in l["runs"]): return False,"missing run keys"
    root_bytes=str(repo_root).encode()
    for seed in gen.LAYOUT_SEEDS:
        for t in IDENTITY_TEMPLATES:
            if root_bytes in (repo_root/t.format(seed=seed)).read_bytes():
                return False,"absolute root leaked"
    return bool(sha256_file(manifest_path)),"manifest complete; deterministic identity scan clean"

def classify(audits):
    failed={x["test"] for x in audits if x["status"]!="PASS"}
    if not failed: return TOKEN
    if failed & {"P4","P13","P14","P17"}: return "MX048_PREFLIGHT_INVALID_SCOPE_OR_PROVENANCE"
    if failed & {"P1","P2","P3"}: return "MX048_PREFLIGHT_BLOCK_LAYOUT_GENERATOR"
    if failed & {"P5","P6"}: return "MX048_PREFLIGHT_BLOCK_RUN_SEED"
    if failed & {"P7","P8","P9","P15"}: return "MX048_PREFLIGHT_BLOCK_CONFIRMATION_SEALING"
    if failed & {"P10","P11","P12"}: return "MX048_PREFLIGHT_BLOCK_ACTUAL_WORLD_BINDING"
    return "MX048_PREFLIGHT_BLOCK_RESERVE_OR_MANIFEST"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--repo-root",type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument("--cross-machine-json",type=Path)
    args=ap.parse_args()
    root=args.repo_root.resolve()
    out=root/RESULT_DIR_REL; out.mkdir(parents=True,exist_ok=True)
    audits=[]
    def rec(name,ok,detail):
        audits.append({"test":name,"status":"PASS" if ok else "FAIL","detail":detail})
        print(f"{name}: {'PASS' if ok else 'FAIL'} - {detail}",flush=True)

    p1,cross=check_p1(root,out,args.cross_machine_json); rec("P1",p1,f"two-root determinism; cross-machine={cross}")
    results=final_generation(root,out)
    write_canonical_json(out/"MX048_GENERATOR_IMPLEMENTATION_PROVENANCE.json",generator_provenance(root))
    ok,d=check_p2(results); rec("P2",ok,d)
    ok,d=check_p3(root,results); rec("P3",ok,d)

    manifest=build_manifest(root,results,datetime.now(timezone.utc).isoformat())
    mp=root/MANIFEST_REL; write_canonical_json(mp,manifest); msha=sha256_file(mp)
    ok,d=check_p4(manifest); rec("P4",ok,d)
    ok,d=check_p5(manifest); rec("P5",ok,d)
    ok,d=check_p6(out,manifest); rec("P6",ok,d)
    ok,d=check_p7(root); rec("P7",ok,d)

    p8,p9,seal=sealing_tests(msha)
    write_canonical_json(out/"MX048_SEALING_PREFLIGHT.json",seal)
    rec("P8",p8,"metadata allowed; raw numeric/GT/ROI/evaluator reads denied")
    rec("P9",p9,"synthetic unseal opens authorized path; incomplete/inconsistent unseal denied")

    ok,d=check_p10(root,mp); (out/"MX048_P10_RESOLVE_ONLY.log").write_text(d+"\n"); rec("P10",ok,"manifest world/config resolves through canonical runner")
    ok,d=check_p11(root,mp); rec("P11",ok,d)
    ok,d=check_p12(root,mp); rec("P12",ok,d)
    ok,d=check_p13(root,manifest); rec("P13",ok,d)
    ok,d=check_p14(root,mp); rec("P14",ok,d)
    ok,d=check_p15(root,seal); rec("P15",ok,d)
    ok,d=check_p16(); rec("P16",ok,d)
    ok,d=check_p17(root); rec("P17",ok,d)
    ok,d=check_p18(root,manifest,mp); rec("P18",ok,d)

    world_rows=[]
    for l in manifest["layouts"]:
        try: verify_layout_identity(root,l); status="PASS"; reason=""
        except Exception as e: status="FAIL"; reason=str(e)
        world_rows.append({
            "layout_seed":l["layout_seed"],"world_path":l["world_path"],
            "world_sha256":l["world_sha256"],"layout_config_sha256":l["layout_config_sha256"],
            "world_identity_sha256":l["world_identity_sha256"],"gt_sha256":l["gt_sha256"],
            "roi_sha256":l["roi_sha256"],"status":status,"reason":reason,
        })
    write_csv(out/"MX048_WORLD_PROVENANCE_PREFLIGHT.csv",world_rows)
    write_csv(out/"MX048_PREFLIGHT_TEST_AUDIT.csv",audits)
    classification=classify(audits)
    result={
        "classification":classification,"all_p1_p18_pass":classification==TOKEN,
        "manifest_path":MANIFEST_REL,"manifest_sha256":msha,
        "technical_repo_commit":git(root,"rev-parse","HEAD"),
        "mx048_contract_commit":gen.MX048_CONTRACT_COMMIT,"cross_machine_detail":cross,
    }
    write_canonical_json(out/"MX048_PREFLIGHT_RESULT.json",result)
    report=[
        "# MX048 implementation / acquisition preflight report","",
        f"- Classification: **{classification}**",
        f"- Technical repo commit: {result['technical_repo_commit']}",
        f"- Contract: {gen.MX048_CONTRACT_COMMIT} / blob {gen.MX048_CONTRACT_BLOB}",
        f"- Manifest SHA256: {msha}",f"- Cross-machine: {cross}","","## P1-P18","",
    ]
    report += [f"- {x['test']}: **{x['status']}** - {x['detail']}" for x in audits]
    report += ["","## Collection gate",""]
    token_path=out/"MX048_PREFLIGHT_TOKEN.txt"
    if classification==TOKEN:
        report.append("Durable token: "+TOKEN); token_path.write_text(TOKEN+"\n",encoding="ascii")
    else:
        report.append("Scientific collection remains blocked."); token_path.unlink(missing_ok=True)
    (out/"MX048_IMPLEMENTATION_REPORT.md").write_text("\n".join(report)+"\n")
    print(classification,flush=True)
    if classification!=TOKEN: raise SystemExit(2)

if __name__=="__main__":
    main()
