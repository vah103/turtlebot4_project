"""Run G2 R7 native2D without touching any other run.

Separate guarded launch because the standard DELL run_guard's *goal* parity
against COM1 Run04 is intentionally invalid after R7 first LOCAL decision.
This guard checks pre-policy source/fingerprint and physical safety resources,
not deliberately different G2 goal selection.
"""
from __future__ import annotations
import os,sys,subprocess,time,json,hashlib,psutil,shutil,signal,datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parent
P36="/home/dell/miniforge3/envs/lama/bin/python"
BASE=Path("/home/dell/DELL_NATIVE2D")
MODE=sys.argv[1] if len(sys.argv)>1 else "check"


def sha(path):
    h=hashlib.sha256()
    with open(path,"rb") as f:
        for part in iter(lambda:f.read(2**20),b""):h.update(part)
    return h.hexdigest()


def write(path,data):
    tmp=path.with_suffix(path.suffix+".partial")
    tmp.write_text(json.dumps(data,indent=2,sort_keys=True)+"\n")
    os.replace(tmp,path)


def verify():
    manifest=json.loads((ROOT/"RUN_MANIFEST.json").read_text())
    assert manifest["id"]=="NR-G2R7-03"
    for rel,expected in manifest["staged_sha256"].items():
        if sha(ROOT/rel)!=expected:
            raise RuntimeError("STAGED_SHA_DRIFT "+rel)
    for path,expected in manifest["source_sha256"].items():
        if sha(path)!=expected:raise RuntimeError("SOURCE_SHA_DRIFT "+path)
    disk=shutil.disk_usage(str(ROOT)).free
    if disk<6*1024**3:raise RuntimeError("DISK_BELOW_6GIB")
    if not Path(P36).exists():raise RuntimeError("PYTHON36_MISSING")
    return {"hashes":"PASS","free_gib":round(disk/1024**3,3),
            "available_mib":round(psutil.virtual_memory().available/1024**2),
            "parity_rule":"BASE_INPUT_G1_PARITY_ONLY; G2 goal parity EXPECTED TO DIFFER"}


def stopper(proc):
    if proc is None or proc.poll() is not None:return
    try:os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:return
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGKILL);proc.wait()


def run(smoke=False):
    note=verify()
    if not smoke:
        proof=ROOT/"MODEL_SMOKE_LATEST.json"
        if not proof.exists() or json.loads(proof.read_text()).get("result")!="PASS":
            raise RuntimeError("MODEL_SMOKE_NOT_PASS")
    if smoke:
        # Keep smoke output away from immutable native full-run directory.
        target=BASE/"smokes"/("NR-G2R7-03_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        target.mkdir(parents=True,exist_ok=False)
        for file in json.loads((ROOT/"RUN_MANIFEST.json").read_text())["staged_sha256"]:
            dest=target/file
            dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(ROOT/file,dest)
    else:target=ROOT
    lock=target/"RUN.lock"
    if lock.exists():raise RuntimeError("RUN_LOCK_EXISTS")
    lock.write_text(str(time.time()))
    if (target/"RUN_RECEIPT.json").exists():raise RuntimeError("OUTPUT_EXISTS")
    env=os.environ.copy()
    env.update({"LD_PRELOAD":"/usr/lib/x86_64-linux-gnu/libstdc++.so.6",
               "DELL_NATIVE2D_ROOT":str(target),
               "DELL_NATIVE2D_THREADS":"2","DELL_NATIVE2D_MODEL_SMOKE":"1" if smoke else "0",
               "OMP_NUM_THREADS":"2","OPENBLAS_NUM_THREADS":"2","MKL_NUM_THREADS":"2",
               "NUMEXPR_NUM_THREADS":"2","OMP_DYNAMIC":"FALSE","MKL_DYNAMIC":"FALSE",
               "PYTHONHASHSEED":"0","MPLBACKEND":"Agg",
               "PYTHONPATH":":".join([str(target),str(BASE/"vendor"),"/home/dell/MapEx",
                                     "/home/dell/MapEx/scripts"]),
               "LC_ALL":"C.UTF-8","LANG":"C.UTF-8"})
    start=time.monotonic()
    proc=None
    reason=None
    peak=0
    lowest=float("inf")
    write(target/"RUN_START.json",{"started":datetime.datetime.now().isoformat(),
          "smoke":smoke, "preflight":note, "profile":"NR-G2R7",
          "policy":"G2_R7_R3_STRUCTURAL_MISSIONS",
          "started_by_user_request":True})
    with (target/"worker.stdout.log").open("wb") as stream:
        try:
            proc=subprocess.Popen([P36,"-u",str(target/"dell_worker.py")],
                                  cwd=str(target),env=env,stdin=subprocess.DEVNULL,
                                  stdout=stream,stderr=subprocess.STDOUT,
                                  preexec_fn=os.setsid)
            print("R7_STARTED",proc.pid,str(target),flush=True)
            guardcount=0
            while proc.poll() is None:
                available=psutil.virtual_memory().available
                lowest=min(lowest,available)
                if available<900*1024**2:guardcount+=1
                else:guardcount=0
                try:
                    parts=[psutil.Process(proc.pid)]+psutil.Process(proc.pid).children(recursive=True)
                    pss=sum(getattr(p.memory_full_info(),"pss",p.memory_info().rss)
                            for p in parts if p.is_running())
                    peak=max(peak,pss)
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
                if guardcount>=2:reason="LOW_AVAILABLE_RAM"
                if shutil.disk_usage(str(ROOT)).free<5*1024**3:reason="LOW_DISK_SPACE"
                if smoke and time.monotonic()-start>600:reason="MODEL_SMOKE_TIMEOUT"
                if reason:
                    print("R7_GUARD_ABORT",reason,flush=True)
                    stopper(proc);break
                time.sleep(3)
            rc=proc.wait()
        except BaseException as e:
            reason=repr(e)
            stopper(proc)
            rc=proc.returncode if proc else -1
    tail=(target/"worker.stdout.log").read_bytes()[-2000:].decode("utf8",errors="replace")
    states=list(target.rglob("mission_status.json"))
    state=json.loads(states[0].read_text()) if states else {}
    complete=state.get("mission_complete") is True
    outcome=("ABORTED_"+reason if reason else
             "MODEL_SMOKE_PASS" if smoke and rc==0 and
                    "DELL_NATIVE2D_MODEL_SMOKE_PASS" in tail else
             "SOURCE_TERMINAL" if not smoke and rc==0 and complete else
             "SOURCE_INCOMPLETE" if not smoke and rc==0 else
             "FAILED")
    receipt=dict(outcome=outcome,returncode=rc,
                 elapsed_s=round(time.monotonic()-start,3),
                 peak_pss_MiB=round(peak/1024**2,2),
                 min_available_MiB=round(lowest/1024**2,2),
                 mission_status=state,reason=reason,mode="smoke" if smoke else "run",
                 log=str(target/"worker.stdout.log"))
    write(target/"RUN_RECEIPT.json",receipt)
    if smoke and outcome=="MODEL_SMOKE_PASS":
        write(ROOT/"MODEL_SMOKE_LATEST.json",{"result":"PASS",
              "receipt":str(target/"RUN_RECEIPT.json")})
    print(json.dumps({k:v for k,v in receipt.items() if k!="mission_status"},
                     indent=2),flush=True)
    if outcome not in ("MODEL_SMOKE_PASS","SOURCE_TERMINAL"):
        raise RuntimeError(outcome)


if __name__=="__main__":
    if MODE=="check":print(json.dumps(verify(),indent=2))
    elif MODE=="smoke":run(smoke=True)
    elif MODE=="run":run(smoke=False)
    else:raise SystemExit("usage: r7_guard.py check|smoke|run")
