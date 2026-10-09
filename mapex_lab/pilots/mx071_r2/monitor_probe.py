"""Monitor a single authorized preflight process and preserve resource-abort evidence."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path("/home/dell/mx071_dell_20261009")
ART = ROOT/"artifacts"


def atomic(path, value):
    tmp=Path(str(path)+".tmp")
    tmp.write_text(json.dumps(value,indent=2,sort_keys=True))
    tmp.replace(path)


def memory():
    values={}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key,val=line.split(":",1)
        values[key]=int(val.strip().split()[0])/1024.
    return values


def rss(pid):
    try:
        for line in Path("/proc/%d/status"%pid).read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])/1024.
    except FileNotFoundError:
        pass
    return 0.


def main():
    name = sys.argv[1] if len(sys.argv)>1 else "cpu_probe"
    if name not in ["cpu_probe","largest_probe","largest_probe_v2","largest_probe_v3"]:
        raise ValueError("UNDECLARED_PROBE")
    manifest=json.loads((ART/"manifest.json").read_text())
    quota=dict(manifest["resource"])
    if name=="largest_probe_v3":
        amendment=json.loads((ART/"resource_quota_v3.json").read_text())
        quota["process_rss_ceiling_mib"]=amendment["process_rss_ceiling_mib"]
    probe=Path(__file__).with_name("preflight.py" if name=="cpu_probe" else "probe_largest.py")
    command=["/home/dell/miniforge3/envs/lama/bin/python",str(probe)]
    if name=="cpu_probe": command.append("probe")
    env=dict(os.environ,PYTHONPATH=str(ROOT/"deps"),
             LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6",MPLBACKEND="Agg")
    started=time.time()
    log=ROOT/"logs"/(name+".log")
    if (ART/(name+"_monitor.json")).exists():
        raise RuntimeError("PROBE_ALREADY_ATTEMPTED: new attempt must have an explicit retry ledger")
    peak=0.
    min_available=float("inf")
    reason=None
    with log.open("w") as stream:
        proc=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT)
        while proc.poll() is None:
            measured=rss(proc.pid)
            available=memory()["MemAvailable"]
            elapsed=time.time()-started
            peak=max(peak,measured)
            min_available=min(min_available,available)
            if measured>quota["process_rss_ceiling_mib"]:
                reason="PROCESS_RSS_CEILING"
            elif available<quota["minimum_available_memory_mib"]:
                reason="SYSTEM_MEMORY_HEADROOM"
            elif elapsed>quota["probe_wall_ceiling_s"]:
                reason="PROBE_WALL_CEILING"
            current=dict(status="RUNNING",pid=proc.pid,elapsed_s=elapsed,
                         peak_rss_mib=peak,minimum_available_memory_mib=min_available,
                         current_rss_mib=measured,current_available_memory_mib=available,
                         command=command,quota=quota)
            atomic(ART/(name+"_progress.json"),current)
            if reason:
                proc.terminate()
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                break
            time.sleep(.5)
    code=proc.wait()
    result=dict(status="RESOURCE_ABORT" if reason else ("PASS" if code==0 else "ERROR"),
                reason=reason,exit_code=code,elapsed_s=time.time()-started,
                peak_rss_mib=peak,minimum_available_memory_mib=min_available,
                log=str(log),quota=quota,
                scientific_trajectories_collected=0)
    atomic(ART/(name+"_monitor.json"),result)
    print(json.dumps(result),flush=True)
    sys.exit(0 if result["status"]=="PASS" else 1)


if __name__=="__main__":
    main()
