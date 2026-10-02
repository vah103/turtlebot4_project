#!/usr/bin/env python3
"""MX028 bounded support-semantics R2 audit.

Reads only the frozen MX028 result CSVs. It does not run inference, simulation,
prediction generation, or scientific recomputation. It verifies that the
Oracle/late summary carries the metric-specific support semantics required by
the IR2 corrective review while preserving predecessor metric values.
"""
from __future__ import annotations
import csv, math
from pathlib import Path

ROOT = Path(__file__).resolve().parent / "mx028_exec_results"
PER = ROOT / "MX028_PXU_PER_DECISION.csv"
SUMMARY = ROOT / "MX028_PXU_ORACLE_LATE_SUMMARY.csv"
ORACLE = {"mpx_001":21,"mpx_002":18,"mpx_003":21,"mpx_004":16,"mpx_005":16,
          "mpx_006":20,"mpx_007":18,"mpx_008":19,"mpx_009":18,"mpx_010":19}

def read(path):
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def finite(v):
    try: return math.isfinite(float(v))
    except (TypeError, ValueError): return False

def med(vals):
    a=sorted(float(v) for v in vals if finite(v))
    if not a: return None
    n=len(a); m=n//2
    return a[m] if n%2 else (a[m-1]+a[m])/2.0

def keys(domain, metric):
    if domain=="TRUTH_FREE" and metric=="OccLowU_share_unknown": return ("D_U_count","")
    if domain=="TRUTH_FREE" and metric=="PredOcc_share_unknown": return ("D_U_count","")
    if domain=="TRUTH_FREE" and metric=="PredOcc_median_U": return ("PredOcc_count","")
    p={"STRUCTURAL_GT":"Structural","LATER_OBSERVED":"Later"}[domain]
    if metric=="OccPrecision_LOW": return (f"{p}_Occ_support_LOW","")
    if metric=="OccPrecision_HIGH": return (f"{p}_Occ_support_HIGH","")
    if metric=="OccPrecision_LowMinusHigh": return (f"{p}_Occ_support_LOW",f"{p}_Occ_support_HIGH")
    if metric=="Occ_U_MedianGap_WrongMinusCorrect": return (f"{p}_Occ_correct_U_n",f"{p}_Occ_wrong_U_n")
    raise KeyError((domain,metric))

def same_num(cell, expected):
    if expected is None: return cell==""
    return finite(cell) and float(cell)==float(expected)

per=read(PER); summary=read(SUMMARY)
assert len(per)==365 and len(summary)==330
byrun={}
for r in per: byrun.setdefault(r["run_id"],[]).append(r)

for out in summary:
    rr=byrun[out["run_id"]]
    if out["scope"]=="ORACLE4":
        ss=[r for r in rr if int(r["decision_id"])==ORACLE[out["run_id"]]]
    elif out["scope"]=="B80_90":
        ss=[r for r in rr if .8<=float(r["normalized_progress"])<.9]
    else:
        ss=[r for r in rr if .9<=float(r["normalized_progress"])<=1.0]
    a,b=keys(out["domain"],out["metric"])
    av=[r[a] for r in ss]; bv=[r[b] for r in ss] if b else []
    ea=float(av[0]) if out["scope"]=="ORACLE4" and finite(av[0]) else med(av)
    eb=(float(bv[0]) if out["scope"]=="ORACLE4" and bv and finite(bv[0]) else med(bv)) if b else None
    assert out["support_primary_label"]==a
    assert out["support_secondary_label"]==b
    assert same_num(out["support_primary_exact_or_median"],ea)
    assert same_num(out["support_secondary_exact_or_median"],eb)

idx={(r["run_id"],r["scope"],r["domain"],r["metric"]):r for r in summary}
assert float(idx[("mpx_001","ORACLE4","TRUTH_FREE","OccLowU_share_unknown")]["support_primary_exact_or_median"])==3045
assert float(idx[("mpx_001","ORACLE4","STRUCTURAL_GT","OccPrecision_LOW")]["support_primary_exact_or_median"])==1192
assert float(idx[("mpx_001","ORACLE4","STRUCTURAL_GT","OccPrecision_HIGH")]["support_primary_exact_or_median"])==1048
g=idx[("mpx_001","ORACLE4","STRUCTURAL_GT","Occ_U_MedianGap_WrongMinusCorrect")]
assert float(g["support_primary_exact_or_median"])==4147
assert float(g["support_secondary_exact_or_median"])==889
print("MX028 support-semantics R2 audit PASS: 330/330 rows")
