"""Phase-aware COM1 resource guard; no other applications are stopped."""
import contextlib
import json
import os
import shutil
import threading
import time
from pathlib import Path
from .contract import CONFIG, atomic_json

def resources():
    def proc_values(path):
        result = {}
        for line in Path(path).read_text().splitlines():
            if ':' in line:
                key, value = line.split(':', 1)
                parts = value.strip().split()
                if parts and parts[0].isdigit():
                    result[key] = int(parts[0]) * (1024 if len(parts)>1 and parts[1]=='kB' else 1)
        return result
    proc = proc_values('/proc/self/status')
    mem = proc_values('/proc/meminfo')
    return {'rss_bytes': proc['VmRSS'], 'rss_peak_bytes': proc['VmHWM'],
            'mem_available_bytes': mem['MemAvailable'],
            'work_free_bytes': shutil.disk_usage('/work').free}

class Guard:
    def __init__(self, out, phase_limit=300, wall_limit=None, quota_roots=None):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.phase_started = self.started
        self.phase_name = 'starting'
        self.phase_limit = phase_limit
        self.wall_limit = wall_limit
        # Count all output artifacts (including CSV, metadata and temporary files).
        roots = sorted(set(Path(p).resolve() for p in (quota_roots or [out])), key=lambda p: len(p.parts))
        self.quota_roots = []
        for root in roots:
            if not any(parent == root or parent in root.parents for parent in self.quota_roots):
                self.quota_roots.append(root)
        self.output_bytes = 0
        self.violation_start = None
        self.timings = []
        self.peak = 0
        self.minimum_mem = 2**63
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *args):
        self.stop.set()
        self.thread.join(timeout=2)
        atomic_json(self.out / 'resource_summary.json', {'rss_peak_bytes': self.peak,
                    'minimum_mem_available_bytes': self.minimum_mem, 'phases': self.timings})

    @contextlib.contextmanager
    def phase(self, name):
        self.phase_name, self.phase_started = name, time.monotonic()
        begin = self.phase_started
        try:
            yield
        finally:
            self.timings.append({'phase': name, 'seconds': time.monotonic()-begin})
            self.phase_name, self.phase_started = 'between_phases', time.monotonic()

    def _loop(self):
        last_heartbeat = 0
        with (self.out / 'resources.jsonl').open('a') as stream:
            while not self.stop.wait(0.5):
                now = time.monotonic()
                r = resources()
                self.peak = max(self.peak, r['rss_bytes'], r['rss_peak_bytes'])
                self.minimum_mem = min(self.minimum_mem, r['mem_available_bytes'])
                r.update(elapsed_s=now-self.started, phase=self.phase_name,
                         phase_elapsed_s=now-self.phase_started)
                stream.write(json.dumps(r, sort_keys=True)+'\n')
                stream.flush()
                reason = None
                bad_mem = (r['rss_bytes'] > CONFIG['rss_ceiling_bytes'] or
                           r['mem_available_bytes'] < CONFIG['mem_available_min_bytes'])
                if bad_mem:
                    if self.violation_start is None:
                        self.violation_start = now
                    if now-self.violation_start >= CONFIG['resource_violation_s']:
                        reason = 'RAM_CEILING'
                else:
                    self.violation_start = None
                if r['work_free_bytes'] < CONFIG['disk_free_min_bytes']:
                    reason = 'DISK_FREE_FLOOR'
                if now-self.phase_started > self.phase_limit:
                    reason = 'PHASE_TIMEOUT'
                if self.wall_limit and now-self.started > self.wall_limit:
                    reason = 'WALL_GUARD'
                if now-last_heartbeat >= CONFIG['heartbeat_s']:
                    total = 0
                    for root in self.quota_roots:
                        for directory, _, files in os.walk(str(root)):
                            for filename in files:
                                try:
                                    total += os.stat(os.path.join(directory, filename)).st_size
                                except FileNotFoundError:  # Atomic rename between listing and stat.
                                    pass
                    self.output_bytes = total
                    r['output_bytes_all_artifacts'] = total
                    if total > CONFIG['output_quota_bytes']:
                        reason = 'OUTPUT_QUOTA_EXCEEDED'
                    atomic_json(self.out / 'heartbeat.json', r)
                    last_heartbeat = now
                if reason:
                    atomic_json(self.out / 'technical_abort.json', dict(r, reason=reason,
                                status='TECHNICAL_ABORT', scientific_execution=False))
                    os._exit(75)
