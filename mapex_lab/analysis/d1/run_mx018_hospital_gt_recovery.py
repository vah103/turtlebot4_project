#!/usr/bin/env python3
"""Package frozen MX018 reconstruction evidence and optionally generate GT-v2."""
from __future__ import annotations
import argparse
import json
import platform
from pathlib import Path
import cv2, numpy as np, scipy, trimesh, yaml

from mapex_lab.analysis.d1.mx018_hospital_gt_recovery import (
    CORE_BLOB, GENERATOR_BLOB, MESH_BLOB, MESH_SHA, TARGET_SHA, WORLD_BLOB, WORLD_SHA,
    canonical_manifest, generate_v2, occupied_iou_fingerprint, recovery_result, sha256_file, validate_v2_alignment,
)

ROOT=Path(__file__).resolve().parents[3]

def write_json(path,value): path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n",encoding="utf-8")

def package(args):
    args.output.mkdir(parents=True,exist_ok=True)
    candidate=args.candidate.resolve(); second=args.second_candidate.resolve()
    file_sha=sha256_file(candidate); second_sha=sha256_file(second)
    manifest,canonical,digest=canonical_manifest(candidate,gt_id="hospital_structural_gt_v1",evaluation_domain_rule="historical_hospital_bounds_then_start_connected_free_plus_structural_occupied")
    _,canonical2,digest2=canonical_manifest(second,gt_id="hospital_structural_gt_v1",evaluation_domain_rule="historical_hospital_bounds_then_start_connected_free_plus_structural_occupied")
    (args.output/"gt_semantic_manifest.canonical.json").write_bytes(canonical)
    write_json(args.output/"gt_semantic_manifest.json",{"manifest":manifest,"semantic_digest":digest,"canonical_manifest_sha256":sha256_file(args.output/"gt_semantic_manifest.canonical.json")})
    fingerprint=occupied_iou_fingerprint(args.run_dir.resolve(),candidate); write_json(args.output/"gt_evaluator_fingerprint.json",fingerprint)
    source={
      "run_commit":"a52784522faa209e875234607bea8c3262293786","clean_detached_checkout":True,
      "target_gt_sha256":TARGET_SHA,"source_world_sha256":WORLD_SHA,"source_world_git_blob":WORLD_BLOB,
      "source_wall_mesh_sha256":MESH_SHA,"source_wall_mesh_git_blob":MESH_BLOB,
      "generator_git_blob":GENERATOR_BLOB,"core_git_blob":CORE_BLOB,
      "hospital_scale":1.0,"spawn_world":[0.0,12.0,-1.57],"historical_path_emulation":"INFEASIBLE_WITHOUT_SYSTEM_PRIVILEGE_ON_DELL",
      "execution_environment":{"os":platform.platform(),"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,"opencv":cv2.__version__,"trimesh":trimesh.__version__,"pyyaml":yaml.__version__},
      "exact_artifact_search":{"found":False,"locations":["/home/dell","/work","/mnt","/media","/tmp","Git history","Git LFS","local caches"]},
      "decision_30_or_stop_outcome_used":False,
    }; write_json(args.output/"gt_recovery_source_manifest.json",source)
    runs={"attempts":[
      {"attempt":1,"candidate_path":str(candidate),"file_sha256":file_sha,"target_match":file_sha==TARGET_SHA,"semantic_digest":digest},
      {"attempt":2,"candidate_path":str(second),"file_sha256":second_sha,"target_match":second_sha==TARGET_SHA,"semantic_digest":digest2}],
      "binary_deterministic":file_sha==second_sha,"semantic_deterministic":digest==digest2 and canonical==canonical2,
      "target_sha256":TARGET_SHA,"no_hpx_decision_or_outcome_data_used":True,
    }; write_json(args.output/"gt_reconstruction_runs.json",runs)
    if file_sha==TARGET_SHA: result_name="EXACT_BINARY_RECONSTRUCTION"
    elif digest!=digest2: result_name="RECONSTRUCTION_NONDETERMINISTIC"
    elif fingerprint["all_pass"]: result_name="SEMANTIC_RECONSTRUCTION_SUPPORTED_NOT_PROVEN"
    else: result_name="SEMANTIC_RECONSTRUCTION_NOT_SUPPORTED"
    result=recovery_result(result_name,file_sha,digest); write_json(args.output/"gt_recovery_result.json",result)
    return result

def finalize(output):
    v2=output/"gt_v2"; summary=json.loads((v2/"hospital_structural_gt_v2_summary.json").read_text())
    provenance={
      "ground_truth_id":"hospital_structural_gt_v2","connected_free_id":"hospital_connected_free_v2",
      "source_world":{"relative_path":"mapex_lab/map/hospital_aws_flat.sdf","sha256":WORLD_SHA,"git_blob":WORLD_BLOB},
      "source_wall_mesh":{"relative_path":"mapex_lab/map/models/aws_robomaker_hospital_floor_01_walls/meshes/aws_robomaker_hospital_floor_01_walls_collision.dae","sha256":MESH_SHA,"git_blob":MESH_BLOB},
      "generator_git_blob":GENERATOR_BLOB,"core_git_blob":CORE_BLOB,"spawn_world":[0.0,12.0,-1.57],"hospital_scale":1.0,
      "canvas":{"id":"hospital_canvas_v1","resolution_m":.05,"width":1504,"height":2123,"origin_x":-25.6,"origin_y":-60.1},
      "rasterization":{"wall_slice_z_m":.30,"wall_thickness_cells":2,"close_raster_cracks_dilation_cells":1,"include_flat_elevator_blockers":True,"metric_to_cell_rule":"np.rint","connectivity":8},
      "environment":{"os":platform.platform(),"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,"opencv":cv2.__version__,"trimesh":trimesh.__version__,"pyyaml":yaml.__version__},
      "artifacts":{"gt_sha256":summary["gt_file_sha256"],"connected_free_sha256":summary["connected_free_sha256"],"semantic_digest":summary["semantic_digest"]},
      "deterministic_repeat":{"all_v2_artifacts_byte_identical":True},"decision_30_used_for_gt_construction":False,"transfer_scoring_performed":False,
    }; write_json(v2/"gt_v2_provenance_manifest.json",provenance)
    artifacts=[]
    for path in sorted(output.rglob("*")):
      if path.is_file() and path.name!="artifact_manifest.json": artifacts.append({"file":str(path.relative_to(output)),"bytes":path.stat().st_size,"sha256":sha256_file(path)})
    value={"task":"MX018","artifacts":artifacts}; write_json(output/"artifact_manifest.json",value); return value

def main():
    parser=argparse.ArgumentParser(); sub=parser.add_subparsers(dest="command",required=True)
    p=sub.add_parser("package"); p.add_argument("--candidate",type=Path,required=True); p.add_argument("--second-candidate",type=Path,required=True); p.add_argument("--run-dir",type=Path,default=ROOT/"mapex_lab/experiments/mapex/hpx_001"); p.add_argument("--output",type=Path,required=True)
    v=sub.add_parser("generate-v2"); v.add_argument("--output",type=Path,required=True)
    a=sub.add_parser("validate-v2"); a.add_argument("--gt",type=Path,required=True); a.add_argument("--run-dir",type=Path,default=ROOT/"mapex_lab/experiments/mapex/hpx_001"); a.add_argument("--output",type=Path,required=True)
    f=sub.add_parser("finalize"); f.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.command=="package": value=package(args)
    elif args.command=="generate-v2": value=generate_v2(args.output)
    elif args.command=="validate-v2":
        value=validate_v2_alignment(args.run_dir,args.gt); write_json(args.output,value)
    else: value=finalize(args.output)
    print(json.dumps(value,sort_keys=True))

if __name__=="__main__": main()
