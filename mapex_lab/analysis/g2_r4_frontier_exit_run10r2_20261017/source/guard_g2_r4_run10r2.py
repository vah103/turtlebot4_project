#!/usr/bin/env python3
"""One fresh guarded MapEx R4.1 full-run pilot. Standalone, on COM1."""
import os, sys, time, json, signal, subprocess, hashlib, shutil, traceback
from pathlib import Path
import psutil
ROOT=Path('/work/com1/mx072_g2_r4_frontier_exit_run10r2_20261017')
RUN="native_g2_r4_frontier_exit_run10r2"
OUTPUT=ROOT/RUN
GUARD=ROOT/"evidence"/"guard_run10r2_frontier_exit"
GUARD.mkdir(parents=True,exist_ok=True)
STATUS=GUARD/"live_status.json"
TERMINAL=GUARD/"terminal_receipt.json"
MAX_SEC=None  # No arbitrary wall-clock cap; normal exploration terminal governs
MAX_OUTPUT_BYTES=None  # No arbitrary output-size cap
MIN_DISK_FREE=5*1024**3  # Emergency filesystem protection only
MIN_MEM_AVAIL=256*1024**2  # Emergency OS protection only
MAX_PSS=None  # No artificial child memory cap
TICK_LIMIT=1000000
LOCK=GUARD/"CLAIM.lock"
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def durable(path,obj):
    tmp=path.with_suffix(path.suffix+".tmp")
    with open(tmp,"w",encoding="utf8") as f:
        json.dump(obj,f,sort_keys=True,indent=2);f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)
def assert_preflight():
    expected={
      'g2_local_backstop.py':'a6abb8eae561fb469f3d6b7a251393a87c7d551d3dbd6b8ff3256a804d7fddd2',
      'g2_predicted_goals.py':'d8eb37a954b47e01f5d2cd313ae9ece439e00a9ce3981f3fe8cdf07af0d9d601',
      'v41src/v4r41.py':'35dd0b22411d235067ee48e859ef9018e1bb184be49034a761a83b728fea4bc9',

      'v41src/v4r4_generator_reference.py':'fb954080339c5cdf9cbfbe57b088983831f8e2aa1728d4d369cce70f64745a7d',

      'explore_g2_r4_lama3_smallroi_run06.py':'b54e51cf36aeff08725e6692b9863f2915aba215d008613940ad0bb67e1625cd',

      'room_native_integration.py':'566a449560a03638af7fc0d736be031bac8e2c84fc4cdab059bf7f64a913354c',

      'g2_native_integration.py':'6f4a90c0663d0f1fe1672a6083cc574355632483850cec1e8af294acf139baeb',

      'g2_r4_policy_sidecar.py':'82244274f00d4e01ba632ae24dcca69b4a775ae89363a191a59fb4f08b470f7f',

      'fast_native_lama_3model.py':'f396346129f3d24de63678e07cced6cee4394222f7f9d9026b83d881b878036d',

      'native_g2_r4_3model_worker_run10r2.py':'9235f48d0d024479c80fa65e8f6ad272d6cdff278c79e59475f9b7a0a712f2ac',

      'sidecar.py':'ed8488a3ab790a774038a703f9c8801d2a5346c672c95f6ea8d6079e68d5215a',

      'g2src/r4_adaptive.py':'6fe606bb409e80b74859f39ff25c8d42c996ea3465fab7684affc62133a13682',

      'g2src/r4_episode.py':'df4850897233d0f368e5be27cb96e008437cebfa8abe3d5f05918e7ee0f405e6',

      'g2src/mapex_g2_r3/commitment.py':'290a03ec536a12b95ed525d496037f7042240fffd38c77d5e7e1c4b12174f370',

      'g2src/mapex_g2_r3/sequence.py':'ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3',

      'g2src/mapex_g2_r3/structural.py':'a09e956dd58c935bb9177f0f888779a494bfafbfbc21c4d03b844fa3a86f7471',

      '../mapex_single_audit/source/MapEx/scripts/explore.py':'2dc4c572141fbe56c335953a126d001d5637fd06a38c2adc66c41bc1028f7fbc',

    }
    observed={}
    for name,value in expected.items():
        path=(ROOT/name).resolve()
        digest=sha(path)
        if digest!=value:raise RuntimeError("PIN_MISMATCH: "+name)
        observed[name]=digest
    assert not OUTPUT.exists(),"output collision"
    assert not TERMINAL.exists(),"terminal collision"
    assert shutil.disk_usage(ROOT).free>=MIN_DISK_FREE
    assert psutil.virtual_memory().available>=512*1024**2,"Only emergency available-RAM preflight"
    for p in psutil.process_iter(["name","cmdline"]):
        try:
            cmd=p.info["cmdline"] or []
            if (p.info["name"] in ("python","python3","python3.12","python3.6") and
                any(part.endswith(("/native_full_worker_maxcpu_v2.py","/native_full_worker.py","/native_plain_mapex_maxcpu_run03_worker.py","/native_opt_r41_3model_worker.py")) for part in cmd[1:]) and
                p.pid!=os.getpid()):
                raise RuntimeError("another full native worker exists")
        except (psutil.AccessDenied,psutil.NoSuchProcess):pass
    return observed
def output_size(path):
    if not path.exists():return 0
    total=0
    for current,dirs,files in os.walk(str(path)):
        for fn in files:
            try:total+=os.stat(os.path.join(current,fn)).st_size
            except FileNotFoundError:pass
    return total
def latest_counts():
    root=OUTPUT/"G2_R4_FRONTIER_EXIT_RUN10R2_visvarprob"
    rr=root/"room_r41"
    try: state=json.loads((root/"mission_status.json").read_text())
    except Exception:state={}
    try:
        last=sorted((rr/"decisions").glob("decision_[0-9]*.json"))
        last_json=json.loads(last[-1].read_text()) if last else {}
    except Exception:last_json={}
    try:scans=len(list((rr/"steps").glob("tick_*_summary.json")))
    except Exception:scans=0
    return state,last_json,scans
def stop(proc,reason):
    print("WATCHDOG_STOP",reason,flush=True)
    try:os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:return
    for i in range(15):
        if proc.poll() is not None:return
        time.sleep(1)
    try:os.killpg(proc.pid,signal.SIGKILL)
    except ProcessLookupError:pass
def main():
    fd=os.open(str(LOCK),os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    with os.fdopen(fd,"w") as f:f.write("claimed_at="+str(time.time())+"\n")
    start=time.time()
    digest=assert_preflight()
    run_metadata={"run":RUN,"kind":"FRESH_NATIVE2D_G2_R4_GOALCHAIN_MATCHED_RUN04_RUN05",
      "user_explicit_go":True,"tick_limit":TICK_LIMIT,"watchdog_limit_s":MAX_SEC,
      "max_output_bytes":MAX_OUTPUT_BYTES,"min_disk_free_bytes":MIN_DISK_FREE,
      "min_mem_available_bytes":MIN_MEM_AVAIL,"max_process_tree_pss_bytes":MAX_PSS,
      "source_sha256":digest,"source_model":"EXACT_RUN04_CPU_G1_G2_G3_CROPPED_LAMA",
      "numerical_parity_with_original":False,
      "effective_cpu_profile":{"torch_threads":16,"torch_interop_threads":16,"OMP_NUM_THREADS":16,"MKL_NUM_THREADS":16,"OPENBLAS_NUM_THREADS":16,"process_nice":0,"cpu_governor":"SYSTEM_UNCHANGED_REQUIRES_ADMIN"},
      "GT_for_online_scoring":False,"QA_independent":False, "R4_version":"old_goal_chain_PLUS_PREDICTED_GOAL_WITH_WFD_EXIT_ONLY", "RoomRun_R41_pinned":True}
    durable(GUARD/"START_MANIFEST.json",run_metadata)
    log=open(GUARD/"native_stdout.log","ab",buffering=0)
    env=dict(os.environ,PYTHONPATH=str(ROOT)+":/work/com1/mapex_single_audit/source/MapEx:/work/com1/mapex_single_audit/source/MapEx/scripts",
        PYTHONHASHSEED="0",OMP_NUM_THREADS="16",OPENBLAS_NUM_THREADS="16",MKL_NUM_THREADS="16",NUMEXPR_NUM_THREADS="16",OMP_DYNAMIC="FALSE",MKL_DYNAMIC="FALSE",LC_ALL="C.UTF-8",LANG="C.UTF-8")
    def child_setup():
        os.setsid()
        # Normal priority 0: remove prior manually reduced CPU priority.
    proc=subprocess.Popen(["/work/conda-envs/mapex-single-audit-com1/bin/python","-u",str(ROOT/"native_g2_r4_3model_worker_run10r2.py")],
        cwd=str(ROOT),env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,preexec_fn=child_setup)
    print("FULL_NATIVE_PID",proc.pid,"START",time.strftime("%F %T"),flush=True)
    stop_reason=None
    alarm_count=0
    last_disk_check=0
    run_bytes=0
    while proc.poll() is None:
        elapsed=time.time()-start
        memory=psutil.virtual_memory().available
        try:
            rootproc=psutil.Process(proc.pid)
            tree=[rootproc]+rootproc.children(recursive=True)
        except psutil.NoSuchProcess:tree=[]
        pss=0
        for p in tree:
            try:pss+=getattr(p.memory_full_info(),"pss",p.memory_info().rss)
            except(psutil.NoSuchProcess,psutil.AccessDenied):pass
        if time.time()-last_disk_check>=30:
            run_bytes=output_size(OUTPUT)
            last_disk_check=time.time()
        free=shutil.disk_usage(ROOT).free
        state,last,scans=latest_counts()
        update={"run":RUN,"state":"RUNNING","worker_pid":proc.pid,"elapsed_s":round(elapsed,1),
          "live_available_mem_mib":round(memory/1024**2),"child_tree_pss_mib":round(pss/1024**2),
          "disk_free_gib":round(free/1024**3,2),"output_gib":round(run_bytes/1024**3,2),
          "native_tick":state.get("cur_step"),"scan_steps":scans,"last_decision":last.get("decision"),
          "last_mode":last.get("mode"),"last_reason":last.get("reason"),"last_room_id":last.get("room_id"),
          "mission_complete":state.get("mission_complete"),"last_update_local":time.strftime("%F %T")}
        durable(STATUS,update)
        if MAX_SEC is not None and elapsed>MAX_SEC: stop_reason="TIME_CAP"
        elif MAX_OUTPUT_BYTES is not None and run_bytes>MAX_OUTPUT_BYTES:stop_reason="OUTPUT_CAP"
        elif free<MIN_DISK_FREE:stop_reason="DISK_FLOOR_10GIB"
        elif MAX_PSS is not None and pss>MAX_PSS:stop_reason="PROCESS_PSS_CAP"
        elif memory<MIN_MEM_AVAIL:
            alarm_count+=1
            if alarm_count>=3:stop_reason="HOST_MEMAVAILABLE_FLOOR_420MIB"
        else:alarm_count=0
        if stop_reason:
            stop(proc,stop_reason)
            break
        time.sleep(6)
    ret=proc.wait()
    log.close()
    state,last,scans=latest_counts()
    outcome=("ABORTED_"+stop_reason if stop_reason else
        "SOURCE_TERMINAL" if ret==0 and state.get("mission_complete") is True else
        "SOURCE_INCOMPLETE" if ret==0 else "EXECUTION_FAILED")
    receipt={"run":RUN,"outcome":outcome,"worker_pid":proc.pid,"returncode":ret,
        "elapsed_s":round(time.time()-start,2),"mission_status":state,"last_decision":last,
        "completed_tick_records":scans,"stop_reason":stop_reason,"finished_local":time.strftime("%F %T"),
        "independent_QA":False,"semantic_accuracy_proven":False}
    durable(TERMINAL,receipt)
    durable(STATUS,{"state":outcome,"run":RUN,"terminal_receipt":str(TERMINAL),
         "scan_steps":scans,"returncode":ret,"updated_local":time.strftime("%F %T")})
    print("WATCHDOG_TERMINAL",outcome,ret,"SCANS",scans,flush=True)
if __name__=="__main__":
    try:main()
    except BaseException as e:
        try:durable(TERMINAL,{"outcome":"GUARD_FAILURE","error":repr(e),"traceback":traceback.format_exc()})
        except Exception:pass
        print("GUARD_FAILURE",repr(e),traceback.format_exc(),flush=True)
        sys.exit(3)
