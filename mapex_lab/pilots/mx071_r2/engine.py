"""Resumable source and first-goal/common-PIPE continuation engine for MX071 R2."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from native_adapter import (
    ROOT,atomic_json,make_mapper,planning_cost,sha_array,sha_file,torch,pyastar2d)
from scoring import score_online
import numpy as np

ART=ROOT/"artifacts"
SEAL=json.loads((ART/"manifest.json").read_text())
MODEL_ID=hashlib.sha256(json.dumps(dict(
    models=SEAL["models"],source=SEAL["source_files"],pipe=SEAL["pipe_commit"],
    preprocessing="default_map_eval float32 input/crop actual centered pad",
    runtime=dict(torch=torch.__version__,numpy=np.__version__,device="cpu",threads=2,worker_seed=0),
    output="channel0,np.mean,torch.var(unbiased=True),K3"),sort_keys=True).encode()).hexdigest()


class ResourceAbort(RuntimeError):
    pass


def available_mib():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1])/1024.


def rss_mib(pid):
    try:
        for line in Path("/proc/%d/status"%pid).read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])/1024.
    except FileNotFoundError:
        pass
    return 0.


def prediction_key(observed):
    return hashlib.sha256((MODEL_ID+sha_array(observed.astype(np.float32))).encode()).hexdigest()


class Predictor:
    def __init__(self,logdir):
        self.cache=ART/"prediction_cache"
        self.cache.mkdir(exist_ok=True)
        self.logdir=Path(logdir)
        self.logdir.mkdir(parents=True,exist_ok=True)
        self.calls=0
        self.member_calls=0
        self.cache_hits=0
        self.member_timings=[]
        self.peak_rss_mib=0.
        self.minimum_available_mib=float("inf")

    def predict(self,observed):
        key=prediction_key(observed)
        target=self.cache/(key+".npz")
        if target.exists():
            with np.load(str(target)) as z:
                if str(z["model_id"])!=MODEL_ID or str(z["key"])!=key:
                    raise RuntimeError("CACHE_IDENTITY_MISMATCH")
                result=tuple(z[x].copy() for x in ["predictions","mean","variance"])
            self.cache_hits+=1
            target.touch()
            return result,key,dict(cache_hit=True,member_timings=[],key=key)
        # Reuse only the verified exact same initial input/model contract.
        initial=np.load(str(ART/"probe_observed.npy"))
        if sha_array(initial)==sha_array(observed.astype(np.float32)):
            with np.load(str(ART/"probe_predictions.npz")) as z:
                result=tuple(z[x].copy() for x in ["predictions","mean","variance"])
            self.cache_hits+=1
            timing=dict(cache_hit=True,preflight_reuse=True,key=key)
        else:
            if shutil.disk_usage(str(ART)).free<512*1024**2:
                raise ResourceAbort("DISK_HEADROOM")
            input_file=self.cache/"worker_input.npy"
            np.save(str(input_file),observed.astype(np.float32))
            members=[]
            timings=[]
            for member in [1,2,3]:
                output_file=self.cache/("worker_G%d.npy"%member)
                timing_file=self.cache/("worker_G%d.json"%member)
                command=["/home/dell/miniforge3/envs/lama/bin/python",
                         str(Path(__file__).with_name("inference_worker.py")),
                         str(member),str(input_file),str(output_file),str(timing_file)]
                env=dict(os.environ,PYTHONPATH=str(ROOT/"deps"),
                         LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6",MPLBACKEND="Agg")
                for attempt in range(2):
                    started=time.time()
                    reason=None
                    with (self.logdir/("inference_G%d.log"%member)).open("a") as log:
                        proc=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
                        while proc.poll() is None:
                            measured=rss_mib(proc.pid)
                            available=available_mib()
                            self.peak_rss_mib=max(self.peak_rss_mib,measured)
                            self.minimum_available_mib=min(self.minimum_available_mib,available)
                            if measured>5120: reason="WORKER_RSS_CEILING"
                            elif available<256: reason="SYSTEM_MEMORY_HEADROOM"
                            elif time.time()-started>1200: reason="INFERENCE_WALL_CEILING"
                            if reason:
                                proc.terminate()
                                try: proc.wait(timeout=10)
                                except subprocess.TimeoutExpired:
                                    proc.kill()
                                    proc.wait()
                                break
                            time.sleep(.5)
                    exit_code=proc.wait()
                    if exit_code==0 and reason is None:
                        break
                    # One same-config retry, retain both attempts in a durable ledger.
                    with (self.logdir/"inference_attempts.jsonl").open("a") as log:
                        log.write(json.dumps(dict(key=key,member=member,attempt=attempt,
                                                  exit_code=exit_code,reason=reason))+"\n")
                    if attempt==1:
                        if reason: raise ResourceAbort(reason)
                        raise RuntimeError("INFERENCE_WORKER_FAILED")
                members.append(np.load(str(output_file)))
                measured=json.loads(timing_file.read_text())
                timings.append(measured)
                self.member_timings.append(measured)
                self.member_calls+=1
            predictions=np.stack(members).astype(np.float32)
            mean=np.mean(predictions,axis=0)
            variance=torch.var(torch.from_numpy(predictions),dim=0).numpy()
            result=(predictions,mean,variance)
            self.calls+=1
            timing=dict(cache_hit=False,member_timings=timings,key=key)
        temp=target.with_suffix(".tmp")
        with temp.open("wb") as f:
            np.savez_compressed(f,predictions=result[0],mean=result[1],variance=result[2],
                                model_id=MODEL_ID,key=key)
        temp.replace(target)
        # Memoization affects compute only. Snapshots store their own immutable arrays.
        files=sorted(self.cache.glob("*.npz"),key=lambda p:p.stat().st_mtime)
        for old in files[:-3]:
            old.unlink()
        return result,key,timing


class State:
    def __init__(self,gt,component,pose,identity):
        self.mapper=make_mapper(gt)
        self.component=component
        self.denominator=int(component.sum())
        self.meta=dict(identity=identity,planning_contract="ALIGNED_SHARED",
                       scorer_contract="P0_NATIVE_SAMPLE_COUNT",
                       model_identity=MODEL_ID,pose=list(map(int,pose)),moves=0,attempts=0,
                       stuck=0,goal=None,last_goal=None,release_reason="INITIAL",
                       decisions=0,prediction_key=None,terminal=None,
                       rng_numpy=json_rng(),rng_torch=torch.get_rng_state().tolist())
        self.meta["prediction_digest"]=None
        self.trace=[]
        self.decision_trace=[]
        self.mapper.observe_and_accumulate_given_pose(self.meta["pose"])
        self.trace.append(self.trace_row())

    def coverage(self):
        return float(np.count_nonzero(self.component & (self.mapper.obs_map==0)))/self.denominator

    def trace_row(self):
        return dict(move=self.meta["moves"],travel_m=self.meta["moves"]/10.,
                    pose=list(self.meta["pose"]),coverage=self.coverage(),
                    observed_hash=sha_array(self.mapper.obs_map),
                    goal=self.meta["goal"],attempts=self.meta["attempts"])

    def complete_hash(self):
        sealed=dict(self.meta,observed_hash=sha_array(self.mapper.obs_map),
                    accum_hit_points_hash=sha_array(self.mapper.accum_hit_points),
                    environment=sha_array(self.mapper.gt_map),
                    component_metadata_hash=sha_array(self.component))
        return hashlib.sha256(json.dumps(sealed,sort_keys=True).encode()).hexdigest()

    def save(self,path):
        meta=dict(self.meta,complete_state_hash=self.complete_hash())
        with Path(path).with_suffix(".tmp").open("wb") as f:
            np.savez_compressed(f,observed=self.mapper.obs_map,
                                accumulated_hits=self.mapper.accum_hit_points,
                                metadata=json.dumps(meta,sort_keys=True),
                                trace=json.dumps(self.trace),
                                decision_trace=json.dumps(self.decision_trace))
        Path(path).with_suffix(".tmp").replace(path)

    @classmethod
    def load(cls,path,gt,component):
        with np.load(str(path)) as z:
            metadata=json.loads(str(z["metadata"]))
            state=cls(gt,component,metadata["pose"],metadata["identity"])
            expected=metadata.pop("complete_state_hash")
            state.meta=metadata
            state.mapper.obs_map=z["observed"].copy()
            state.mapper.accum_hit_points=z["accumulated_hits"].copy()
            state.trace=json.loads(str(z["trace"]))
            state.decision_trace=json.loads(str(z["decision_trace"]))
        if state.complete_hash()!=expected:
            raise RuntimeError("CLONE_COMPLETE_STATE_HASH_MISMATCH")
        restore_rng(state.meta)
        return state


def json_rng():
    kind,keys,pos,gauss,value=np.random.get_state()
    return [kind,keys.tolist(),pos,gauss,value]


def restore_rng(meta):
    r=meta["rng_numpy"]
    np.random.set_state((r[0],np.asarray(r[1],dtype=np.uint32),r[2],r[3],r[4]))
    torch.set_rng_state(torch.tensor(meta["rng_torch"],dtype=torch.uint8))


def valid_goal(state,cost):
    goal=state.meta["goal"]
    if goal is None: return False,"NO_ACTIVE_GOAL"
    if not np.isfinite(cost[tuple(goal)]): return False,"INVALIDATED_INFLATED"
    if np.linalg.norm(np.asarray(goal)-state.meta["pose"])<10: return False,"REACHED_TOLERANCE"
    path=pyastar2d.astar_path(cost,tuple(state.meta["pose"]),tuple(goal),allow_diagonal=False)
    if path is None: return False,"UNREACHABLE_RELEASE"
    return True,path


def replay_key(state,goal):
    payload=dict(complete_state_hash=state.complete_hash(),goal=list(map(int,goal)),
                 continuation="PIPE_ALIGNED",commitment="until validity/unreachable/reached release",
                 controller="one 4connected cell then source physical scan",
                 budget_remaining_moves=1000-state.meta["moves"],
                 model_identity=MODEL_ID,execution_contract_hash=state.meta.get("execution_contract_hash"))
    return hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()


def execute(state,predictor,on_decision=None,on_progress=None,first_goal=None,move_ceiling=1000):
    """One intervention, then common PIPE; GT used only by environment/evaluator."""
    intervention=None
    start_move=state.meta["moves"]
    initial_unknown=state.mapper.obs_map==.5
    if first_goal is not None:
        state.meta["goal"]=list(map(int,first_goal))
        state.meta["decisions"]+=1
    intervention_active=first_goal is not None
    while state.meta["moves"]<move_ceiling and state.meta["terminal"] is None:
        cost=planning_cost(state.mapper,"ALIGNED_SHARED")
        valid,path_or_reason=valid_goal(state,cost)
        if not valid:
            if intervention_active:
                intervention=dict(end_move=state.meta["moves"],release_reason=path_or_reason,
                                  new_observed=(state.mapper.obs_map!=.5)&initial_unknown,
                                  poses=[r["pose"] for r in state.trace if r["move"]>=start_move])
                intervention_active=False
            state.meta["last_goal"]=state.meta["goal"]
            state.meta["goal"]=None
            state.meta["release_reason"]=path_or_reason
            phase="INFERENCE"
            try:
                (members,mean,variance),key,timing=predictor.predict(state.mapper.obs_map)
                state.meta["prediction_key"]=key
                state.meta["prediction_digest"]=[sha_array(x) for x in [members,mean,variance]]
                state.meta["rng_numpy"]=json_rng()
                state.meta["rng_torch"]=torch.get_rng_state().tolist()
                phase="PLANNER"
                online=score_online(state.mapper.obs_map,np.asarray(state.meta["pose"]),
                                    cost,members,mean,variance)
                if on_decision:
                    on_decision(state,online,(members,mean,variance),timing)
            except ResourceAbort as exc:
                state.meta["terminal"]="RESOURCE_ABORT"
                state.meta["terminal_detail"]=str(exc)
                break
            except Exception as exc:
                state.meta["terminal"]="INFERENCE_ERROR" if phase=="INFERENCE" else "PLANNER_ERROR"
                state.meta["terminal_detail"]=type(exc).__name__+": "+str(exc)
                break
            if online["chosen"] is None:
                state.meta["terminal"]="NO_REACHABLE_FRONTIER"
                break
            chosen=online["chosen"]
            state.decision_trace.append(dict(
                move=state.meta["moves"],complete_state_hash=state.complete_hash(),
                goal=chosen["goal"].tolist(),release=path_or_reason,
                path_hash=sha_array(chosen["path"]),scores=[c["P0"] for c in online["pool"]],
                observed_known_hex=np.packbits((state.mapper.obs_map!=.5).ravel()).tobytes().hex(),
                predict_key=key))
            state.meta["goal"]=chosen["goal"].tolist()
            state.meta["decisions"]+=1
            valid,path_or_reason=valid_goal(state,cost)
            if not valid:
                state.meta["terminal"]="INVALID_SELECTED_GOAL"
                break
        path=path_or_reason
        if len(path)<2:
            state.meta["goal"]=None
            state.meta["stuck"]+=1
        else:
            next_pose=path[1].astype(int)
            state.meta["attempts"]+=1
            if state.mapper.gt_map[tuple(next_pose)]==1:
                state.meta["terminal"]="COLLISION"
                break
            prev=state.meta["pose"]
            if np.sum(np.abs(next_pose-np.asarray(prev)))!=1:
                state.meta["terminal"]="NON_4CONNECTED_CONTROLLER_ERROR"
                break
            state.meta["pose"]=next_pose.tolist()
            state.meta["moves"]+=1
            state.mapper.observe_and_accumulate_given_pose(next_pose)
            state.meta["stuck"]=0
            state.trace.append(state.trace_row())
        if state.meta["stuck"]>=50:
            state.meta["terminal"]="STUCK"
            break
        if on_progress:
            on_progress(state)
    if state.meta["terminal"] is None:
        state.meta["terminal"]="BUDGET_EXHAUSTED"
    if intervention_active:
        intervention=dict(end_move=state.meta["moves"],release_reason=state.meta["terminal"],
                          new_observed=(state.mapper.obs_map!=.5)&initial_unknown,
                          poses=[r["pose"] for r in state.trace if r["move"]>=start_move])
    return intervention


def branch_metrics(trace,start_move,terminal):
    rows=[r for r in trace if r["move"]>=start_move]
    if not rows:
        raise RuntimeError("MISSING_BASELINE_TRACE")
    remaining=(1000-start_move)/10.
    c0=rows[0]["coverage"]
    points=[(r["move"]/10.-start_move/10.,r["coverage"]-c0) for r in rows]
    if points[-1][0]<remaining:
        points.append((remaining,points[-1][1]))
    # Coverage is a right-continuous step after each 0.1m motion/scan.
    area=sum((points[i+1][0]-points[i][0])*points[i][1] for i in range(len(points)-1))
    prefix=next((r["coverage"]-c0 for r in rows if r["move"]>=start_move+100),
                rows[-1]["coverage"]-c0)
    return dict(Q=None if terminal in ["RESOURCE_ABORT","INFERENCE_ERROR","PLANNER_ERROR"] else area/remaining,
                prefix_gain_10m=prefix,remaining_m=remaining,terminal=terminal,
                terminal_coverage=rows[-1]["coverage"],coverage_start=c0,
                behavior_failure=terminal not in ["BUDGET_EXHAUSTED","NO_REACHABLE_FRONTIER"],
                actual_travel_m=rows[-1]["travel_m"]-start_move/10.)
