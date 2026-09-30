#!/usr/bin/env python3
from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

BASE_SHA = "4899ee95c85640965befeaf98c20f156c0d3181a"
METHOD_SHA = "fc0ea05313dc02e6b8a957552c31f952de0897d6"
LAB = Path(__file__).resolve().parents[1]
EXP = LAB / "experiments" / "mapex"
SHARED = LAB / "analysis" / "d1" / "results" / "shared_phase0_evidence_v1" / "shared_phase0_evidence.csv"
OUT = LAB / "analysis" / "mx025_exec_results"
ORACLE = {
    "mpx_001": 21, "mpx_002": 18, "mpx_003": 21, "mpx_004": 16, "mpx_005": 16,
    "mpx_006": 20, "mpx_007": 18, "mpx_008": 19, "mpx_009": 18, "mpx_010": 19,
}
RUNS = list(ORACLE)

def read_csv(path):
    with Path(path).open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))

def write_csv(path, rows):
    rows = list(rows)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with Path(path).open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

def scalar(z, key):
    v = np.asarray(z[key]).reshape(()).item()
    return v.decode("utf-8") if isinstance(v, bytes) else v

def exact_int(v, name):
    x = float(v)
    if not math.isfinite(x) or int(x) != x:
        raise ValueError(name + "_NONINTEGRAL")
    return int(x)

def finite(v):
    try:
        x = float(v)
    except Exception:
        return math.nan
    return x if math.isfinite(x) else math.nan

def is_true(v):
    return str(v).strip().lower() in {"1", "true", "yes"}

def qlinear(vals, q):
    a = np.asarray(list(vals), dtype=float)
    a = a[np.isfinite(a)]
    return float(np.quantile(a, q, method="linear")) if a.size else math.nan

def median(vals):
    return qlinear(vals, 0.5)

def resolve(run, rel):
    p = (run / rel).resolve()
    if run.resolve() not in p.parents:
        raise ValueError("PATH_ESCAPES_RUN")
    return p

def load_raw(path, did):
    if path.name != f"decision_{did:06d}_raw.npz":
        raise ValueError("RAW_IDENTITY_FAIL")
    with np.load(path, allow_pickle=False) as z:
        need = ("data","resolution","width","height","origin_x","origin_y","origin_yaw","frame_id","source_stamp_s")
        missing = [k for k in need if k not in z.files]
        if missing:
            raise ValueError("RAW_MISSING_META_" + "_".join(missing))
        data = np.asarray(z["data"])
        meta = {k: scalar(z, k) for k in need if k != "data"}
    if data.ndim != 2:
        raise ValueError("RAW_NOT_2D")
    h = exact_int(meta["height"], "RAW_HEIGHT")
    w = exact_int(meta["width"], "RAW_WIDTH")
    if data.shape != (h, w):
        raise ValueError("RAW_SHAPE_FAIL")
    if not np.all(np.isfinite(data)):
        raise ValueError("RAW_NONFINITE")
    if not math.isfinite(float(meta["resolution"])) or float(meta["resolution"]) <= 0:
        raise ValueError("RAW_RESOLUTION_FAIL")
    for key in ("origin_x","origin_y","origin_yaw","source_stamp_s"):
        if not math.isfinite(float(meta[key])):
            raise ValueError("RAW_" + key.upper() + "_NONFINITE")
    if float(meta["origin_yaw"]) != 0.0:
        raise ValueError("RAW_ORIGIN_YAW_FAIL")
    if str(meta["frame_id"]) != "map":
        raise ValueError("RAW_FRAME_FAIL")
    return data, meta

def load_member(path, did, member, raw, rawmeta, crop0=None):
    if path.name != f"decision_{did:06d}_{member.lower()}.npz":
        raise ValueError(member + "_IDENTITY_FAIL")
    with np.load(path, allow_pickle=False) as z:
        need = ("data","source_height","source_width","pad_top","pad_left","member",
                "source_map_stamp_s","resolution","origin_x","origin_y")
        missing = [k for k in need if k not in z.files]
        if missing:
            raise ValueError(member + "_MISSING_META_" + "_".join(missing))
        data = np.asarray(z["data"])
        meta = {k: scalar(z, k) for k in need if k != "data"}
    if data.ndim != 2:
        raise ValueError(member + "_NOT_2D")
    h = exact_int(meta["source_height"], "SOURCE_HEIGHT")
    w = exact_int(meta["source_width"], "SOURCE_WIDTH")
    t = exact_int(meta["pad_top"], "PAD_TOP")
    l = exact_int(meta["pad_left"], "PAD_LEFT")
    if (h, w) != raw.shape:
        raise ValueError(member + "_SOURCE_SHAPE_FAIL")
    if min(t, l) < 0 or t + h > data.shape[0] or l + w > data.shape[1]:
        raise ValueError(member + "_CROP_BOUNDS_FAIL")
    if str(meta["member"]) != member:
        raise ValueError(member + "_MEMBER_FAIL")
    if meta["source_map_stamp_s"] != rawmeta["source_stamp_s"]:
        raise ValueError(member + "_STAMP_FAIL")
    if meta["resolution"] != rawmeta["resolution"] or meta["origin_x"] != rawmeta["origin_x"] or meta["origin_y"] != rawmeta["origin_y"]:
        raise ValueError(member + "_GEOMETRY_FAIL")
    crop = (h, w, t, l)
    if crop0 is not None and crop != crop0:
        raise ValueError(member + "_CROP_METADATA_MISMATCH")
    runtime = np.asarray(data[t:t+h, l:l+w])
    if runtime.shape != raw.shape:
        raise ValueError(member + "_CROP_SHAPE_FAIL")
    if not np.all(np.isfinite(runtime)):
        raise ValueError(member + "_NONFINITE")
    if np.any(runtime < 0) or np.any(runtime > 1):
        raise ValueError(member + "_RANGE_FAIL")
    return runtime, crop, meta, data.shape

def support_label(n):
    if n == 0:
        return "NO_SUPPORT"
    if n == 1:
        return "LOW_SUPPORT_1"
    return f"VALID_N_{n}"

def direction(x):
    if not math.isfinite(x):
        return "NA"
    if x < 0:
        return "DECREASING"
    if x > 0:
        return "INCREASING"
    return "EXACT_ZERO"

def spearman(xs, ys):
    if len(xs) < 5:
        return math.nan, len(xs), "INSUFFICIENT_PAIRED_LT5"
    r = spearmanr(xs, ys, nan_policy="raise")
    return float(r.statistic), len(xs), ""

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    shared_rows = read_csv(SHARED)
    shared = {(r["run_id"], int(r["decision_id"])): r for r in shared_rows}
    if len(shared) != 365:
        raise RuntimeError(f"SHARED_MEMBERSHIP_FAIL:{len(shared)}")

    per_decision = []
    source_parity = []
    paired = []
    recovery = []
    invariant_violations = []

    for run_id in RUNS:
        run = EXP / run_id
        decisions = sorted(read_csv(run / "decisions.csv"), key=lambda r: int(r["decision_id"]))
        n = len(decisions)
        for row in decisions:
            did = int(row["decision_id"])
            dp = (did - ORACLE[run_id]) / (n - 1)
            band = "ANCHOR" if did == ORACLE[run_id] else ("PRE" if -0.10 <= dp < 0 else ("POST" if 0 < dp <= 0.10 else ""))
            in_w10 = int(abs(dp) <= 0.10 + 1e-12)
            in_w20 = int(abs(dp) <= 0.20 + 1e-12)
            paths = {k: resolve(run, row[k]) for k in ("raw_map","g1_map","g2_map","g3_map")}
            sr = shared[(run_id, did)]

            out = {
                "run_id": run_id, "decision_id": did, "decision_count": n,
                "progress": (did - 1) / (n - 1), "delta_p": dp, "band": band,
                "in_W10": in_w10, "in_W20": in_w20,
                "R_MAP_evaluable": 0, "R_MAP_reason": "",
                "raw_total_count": "", "MAPEX_UNKNOWN_VALID_count": "",
                "KnownFree_map_count": "", "KnownFree_map_m2": "",
                "R_MAP_R1_count": "", "R_MAP_R2_count": "", "R_MAP_R3_count": "",
                "R_MAP_A1_m2": "", "R_MAP_A2_m2": "", "R_MAP_A3_m2": "",
                "R_MAP_A_mean_m2": "", "R_MAP_A_min_m2": "", "R_MAP_A_max_m2": "", "R_MAP_A_range_m2": "",
                "MapRemainingFraction": "",
                "R_REACHABLE_evaluable": int(is_true(sr.get("d1_runtime_region_evaluable",""))),
                "R_REACHABLE_reason": sr.get("d1_reason",""),
                "R_REACHABLE_KnownReachableFree_m2": finite(sr.get("KnownReachableFree_m2")),
                "R_REACHABLE_A1_m2": finite(sr.get("A1_m2")),
                "R_REACHABLE_A2_m2": finite(sr.get("A2_m2")),
                "R_REACHABLE_A3_m2": finite(sr.get("A3_m2")),
                "R_REACHABLE_A_mean_m2": finite(sr.get("A_mean_m2")),
                "R_REACHABLE_RemainingFraction": finite(sr.get("RemainingFraction")),
            }
            parity = {
                "run_id": run_id, "decision_id": did,
                "raw_identity": row["raw_map"], "g1_identity": row["g1_map"], "g2_identity": row["g2_map"], "g3_identity": row["g3_map"],
                "raw_sha256": digest(paths["raw_map"]), "g1_sha256": digest(paths["g1_map"]),
                "g2_sha256": digest(paths["g2_map"]), "g3_sha256": digest(paths["g3_map"]),
                "alignment_pass": 0, "reason": "",
            }
            try:
                raw, rm = load_raw(paths["raw_map"], did)
                crop = None
                members = []
                for key, member in (("g1_map","G1"),("g2_map","G2"),("g3_map","G3")):
                    a, c, meta, padded_shape = load_member(paths[key], did, member, raw, rm, crop)
                    if crop is None:
                        crop = c
                    members.append(a)
                parity.update(
                    alignment_pass=1, reason="",
                    source_height=crop[0], source_width=crop[1], pad_top=crop[2], pad_left=crop[3],
                    resolution=rm["resolution"], origin_x=rm["origin_x"], origin_y=rm["origin_y"],
                    source_stamp_s=rm["source_stamp_s"], runtime_shape=f"{raw.shape[0]}x{raw.shape[1]}",
                )
                unknown = raw < 0
                knownfree = raw == 0
                area_cell = float(rm["resolution"]) ** 2
                regions = [unknown & (m < 0.5) for m in members]
                counts = [int(np.count_nonzero(x)) for x in regions]
                areas = [c * area_cell for c in counts]
                knownfree_count = int(np.count_nonzero(knownfree))
                knownfree_area = knownfree_count * area_cell
                amean = float(np.mean(areas))
                denom = knownfree_area + amean
                if denom == 0:
                    frac = math.nan
                    reason = "ZERO_MAP_FREE_DENOMINATOR"
                    evaluable = 0
                else:
                    frac = amean / denom
                    reason = ""
                    evaluable = 1
                out.update(
                    R_MAP_evaluable=evaluable, R_MAP_reason=reason,
                    raw_total_count=int(raw.size), MAPEX_UNKNOWN_VALID_count=int(np.count_nonzero(unknown)),
                    KnownFree_map_count=knownfree_count, KnownFree_map_m2=knownfree_area,
                    R_MAP_R1_count=counts[0], R_MAP_R2_count=counts[1], R_MAP_R3_count=counts[2],
                    R_MAP_A1_m2=areas[0], R_MAP_A2_m2=areas[1], R_MAP_A3_m2=areas[2],
                    R_MAP_A_mean_m2=amean, R_MAP_A_min_m2=min(areas), R_MAP_A_max_m2=max(areas),
                    R_MAP_A_range_m2=max(areas)-min(areas), MapRemainingFraction=frac,
                )

                reach_valid = bool(out["R_REACHABLE_evaluable"])
                if reach_valid:
                    reach_members = [out["R_REACHABLE_A1_m2"], out["R_REACHABLE_A2_m2"], out["R_REACHABLE_A3_m2"]]
                    for j, (ma, ra) in enumerate(zip(areas, reach_members), 1):
                        if not math.isfinite(float(ra)):
                            invariant_violations.append((run_id, did, j, ma, ra, "REACHABLE_MEMBER_NONFINITE_WHILE_EVALUABLE"))
                        elif ma + 1e-12 < float(ra):
                            invariant_violations.append((run_id, did, j, ma, ra, "MAP_LT_REACHABLE"))
                    if not all(math.isfinite(float(x)) for x in (out["R_REACHABLE_A_mean_m2"], out["R_REACHABLE_RemainingFraction"])):
                        raise RuntimeError(f"{run_id}/{did}: reachable evaluable but comparator nonfinite")

                if evaluable and reach_valid:
                    area_map = amean
                    area_reach = float(out["R_REACHABLE_A_mean_m2"])
                    frac_map = frac
                    frac_reach = float(out["R_REACHABLE_RemainingFraction"])
                    area_ratio = area_map / area_reach if area_reach > 0 else math.nan
                    area_ratio_reason = "" if area_reach > 0 else "ZERO_REACHABLE_AREA_DENOMINATOR"
                    frac_ratio = frac_map / frac_reach if frac_reach > 0 else math.nan
                    frac_ratio_reason = "" if frac_reach > 0 else "ZERO_REACHABLE_FRACTION_DENOMINATOR"
                    paired.append({
                        "run_id": run_id, "decision_id": did, "decision_count": n,
                        "delta_p": dp, "band": band, "in_W10": in_w10, "in_W20": in_w20,
                        "A_map_mean_m2": area_map, "A_reachable_mean_m2": area_reach,
                        "A_signed_difference_m2": area_map - area_reach,
                        "A_ratio_map_over_reachable": area_ratio, "A_ratio_reason": area_ratio_reason,
                        "MapRemainingFraction": frac_map, "RemainingFraction_reachable": frac_reach,
                        "Fraction_signed_difference": frac_map - frac_reach,
                        "Fraction_ratio_map_over_reachable": frac_ratio, "Fraction_ratio_reason": frac_ratio_reason,
                        "KnownFree_map_m2": knownfree_area,
                        "KnownReachableFree_m2": float(out["R_REACHABLE_KnownReachableFree_m2"]),
                        "member1_map_minus_reachable_m2": areas[0] - float(out["R_REACHABLE_A1_m2"]),
                        "member2_map_minus_reachable_m2": areas[1] - float(out["R_REACHABLE_A2_m2"]),
                        "member3_map_minus_reachable_m2": areas[2] - float(out["R_REACHABLE_A3_m2"]),
                    })
                elif in_w10 and evaluable and not reach_valid:
                    recovery.append({
                        "run_id": run_id, "decision_id": did, "decision_count": n,
                        "delta_p": dp, "band": band,
                        "R_MAP_A_mean_m2": amean, "MapRemainingFraction": frac,
                        "reachable_reason": out["R_REACHABLE_reason"] or "UNKNOWN_REACHABLE_NA_REASON",
                    })
            except Exception as exc:
                reason = f"SOURCE_VALIDATION_FAIL:{type(exc).__name__}:{exc}"
                out["R_MAP_reason"] = reason
                parity["reason"] = reason
            per_decision.append(out)
            source_parity.append(parity)

    if len(per_decision) != 365 or len(source_parity) != 365:
        raise RuntimeError("PRIMARY_ROW_MEMBERSHIP_FAIL")
    if invariant_violations:
        raise RuntimeError("SUBSET_INVARIANT_FAIL:" + json.dumps(invariant_violations[:10]))

    write_csv(OUT / "MX025_R_MAP_PER_DECISION.csv", per_decision)
    write_csv(OUT / "MX025_R_MAP_SOURCE_PARITY.csv", source_parity)
    write_csv(OUT / "MX025_R_MAP_VS_REACHABLE_PAIRED.csv", paired)
    write_csv(OUT / "MX025_R_MAP_MISSINGNESS_RECOVERY.csv", recovery)

    per_run = []
    for run_id in RUNS:
        rows = [r for r in per_decision if r["run_id"] == run_id and r["in_W10"] and r["R_MAP_evaluable"]]
        prow = [r for r in paired if r["run_id"] == run_id and r["in_W10"]]
        recovered = [r for r in recovery if r["run_id"] == run_id]
        row_out = {"run_id": run_id}
        for metric in ("MapRemainingFraction","R_MAP_A_mean_m2"):
            pre = [float(r[metric]) for r in rows if r["band"] == "PRE"]
            anc = [float(r[metric]) for r in rows if r["band"] == "ANCHOR"]
            post = [float(r[metric]) for r in rows if r["band"] == "POST"]
            pmed = median(pre)
            aval = anc[0] if len(anc) == 1 else math.nan
            smed = median(post)
            ap = aval - pmed if math.isfinite(aval) and math.isfinite(pmed) else math.nan
            sp = smed - pmed if math.isfinite(smed) and math.isfinite(pmed) else math.nan
            prefix = "Fraction" if metric == "MapRemainingFraction" else "Area"
            row_out.update({
                prefix + "_PRE_values": ";".join(repr(x) for x in pre),
                prefix + "_PRE_n": len(pre), prefix + "_PRE_support": support_label(len(pre)), prefix + "_PRE_median": pmed,
                prefix + "_ANCHOR_n": len(anc), prefix + "_ANCHOR": aval,
                prefix + "_POST_values": ";".join(repr(x) for x in post),
                prefix + "_POST_n": len(post), prefix + "_POST_support": support_label(len(post)), prefix + "_POST_median": smed,
                prefix + "_anchor_minus_PRE": ap, prefix + "_POST_minus_PRE": sp,
                prefix + "_anchor_direction": direction(ap), prefix + "_post_direction": direction(sp),
            })
        area_x = [float(r["A_map_mean_m2"]) for r in prow]
        area_y = [float(r["A_reachable_mean_m2"]) for r in prow]
        frac_x = [float(r["MapRemainingFraction"]) for r in prow]
        frac_y = [float(r["RemainingFraction_reachable"]) for r in prow]
        ar, an, ar_reason = spearman(area_x, area_y)
        fr, fn, fr_reason = spearman(frac_x, frac_y)
        row_out.update(
            W10_R_MAP_valid_n=len(rows), W10_paired_n=len(prow), W10_recovered_from_reachable_NA=len(recovered),
            W10_area_spearman=ar, W10_area_spearman_n=an, W10_area_spearman_reason=ar_reason,
            W10_fraction_spearman=fr, W10_fraction_spearman_n=fn, W10_fraction_spearman_reason=fr_reason,
        )
        per_run.append(row_out)
    write_csv(OUT / "MX025_R_MAP_PER_RUN.csv", per_run)

    synthesis = []
    for metric, prefix in (("MapRemainingFraction","Fraction"),("R_MAP_A_mean_m2","Area")):
        valid_anchor = 0
        paired_sides = 0
        anchor_dirs = Counter()
        post_dirs = Counter()
        anchor_changes = []
        post_changes = []
        for r in per_run:
            aval = finite(r[prefix + "_ANCHOR"])
            pre_n = int(r[prefix + "_PRE_n"])
            post_n = int(r[prefix + "_POST_n"])
            if math.isfinite(aval):
                valid_anchor += 1
            if pre_n >= 1 and post_n >= 1:
                paired_sides += 1
            ad = r[prefix + "_anchor_direction"]
            pd = r[prefix + "_post_direction"]
            anchor_dirs[ad] += 1
            post_dirs[pd] += 1
            ac = finite(r[prefix + "_anchor_minus_PRE"])
            pc = finite(r[prefix + "_POST_minus_PRE"])
            if math.isfinite(ac):
                anchor_changes.append(ac)
            if math.isfinite(pc):
                post_changes.append(pc)
        synthesis.append({
            "section": "PRIMARY_ORACLE_LOCAL", "metric": metric,
            "valid_anchor_runs": valid_anchor, "runs_with_PRE_and_POST": paired_sides,
            "PRE_to_ANCHOR_decreasing": anchor_dirs["DECREASING"], "PRE_to_ANCHOR_increasing": anchor_dirs["INCREASING"],
            "PRE_to_ANCHOR_exact_zero": anchor_dirs["EXACT_ZERO"], "PRE_to_ANCHOR_NA": anchor_dirs["NA"],
            "PRE_to_POST_decreasing": post_dirs["DECREASING"], "PRE_to_POST_increasing": post_dirs["INCREASING"],
            "PRE_to_POST_exact_zero": post_dirs["EXACT_ZERO"], "PRE_to_POST_NA": post_dirs["NA"],
            "run_macro_median_anchor_minus_PRE": median(anchor_changes),
            "run_macro_median_POST_minus_PRE": median(post_changes),
        })

    w10_map_valid = [r for r in per_decision if r["in_W10"] and r["R_MAP_evaluable"]]
    w10_reach_valid = [r for r in per_decision if r["in_W10"] and r["R_REACHABLE_evaluable"]]
    w10_total = [r for r in per_decision if r["in_W10"]]
    recovery_reasons = Counter(r["reachable_reason"] for r in recovery)
    synthesis.append({
        "section": "MISSINGNESS_RECOVERY", "metric": "W10_availability",
        "W10_rows": len(w10_total), "R_MAP_valid_rows": len(w10_map_valid),
        "R_REACHABLE_valid_rows": len(w10_reach_valid),
        "R_MAP_valid_R_REACHABLE_NA_recovered_rows": len(recovery),
        "recovery_reasons_json": json.dumps(dict(sorted(recovery_reasons.items())), sort_keys=True),
    })

    w10_paired = [r for r in paired if r["in_W10"]]
    area_diffs = [float(r["A_signed_difference_m2"]) for r in w10_paired]
    area_ratios = [finite(r["A_ratio_map_over_reachable"]) for r in w10_paired]
    area_ratios = [x for x in area_ratios if math.isfinite(x)]
    frac_diffs = [float(r["Fraction_signed_difference"]) for r in w10_paired]
    frac_ratios = [finite(r["Fraction_ratio_map_over_reachable"]) for r in w10_paired]
    frac_ratios = [x for x in frac_ratios if math.isfinite(x)]
    synthesis.append({
        "section": "PAIRED_W10", "metric": "map_vs_reachable",
        "paired_rows": len(w10_paired),
        "A_signed_difference_m2_median": median(area_diffs),
        "A_ratio_map_over_reachable_median": median(area_ratios),
        "A_ratio_valid_n": len(area_ratios),
        "Fraction_signed_difference_median": median(frac_diffs),
        "Fraction_ratio_map_over_reachable_median": median(frac_ratios),
        "Fraction_ratio_valid_n": len(frac_ratios),
    })
    write_csv(OUT / "MX025_R_MAP_ORACLE_SYNTHESIS.csv", synthesis)

    m4 = [r for r in per_decision if r["run_id"] == "mpx_004" and r["in_W10"]]
    m4_table = []
    for r in m4:
        m4_table.append({
            "run_id": r["run_id"], "decision_id": r["decision_id"], "delta_p": r["delta_p"], "band": r["band"],
            "R_MAP_A_mean_m2": r["R_MAP_A_mean_m2"], "KnownFree_map_m2": r["KnownFree_map_m2"],
            "MapRemainingFraction": r["MapRemainingFraction"],
            "R_REACHABLE_evaluable": r["R_REACHABLE_evaluable"],
            "R_REACHABLE_A_mean_m2": r["R_REACHABLE_A_mean_m2"],
            "R_REACHABLE_KnownReachableFree_m2": r["R_REACHABLE_KnownReachableFree_m2"],
            "R_REACHABLE_RemainingFraction": r["R_REACHABLE_RemainingFraction"],
            "R_REACHABLE_reason": r["R_REACHABLE_reason"],
        })
    write_csv(OUT / "MX025_R_MAP_MPX004_W10_SANITY.csv", m4_table)

    dictionary = {
        "task": "MX025",
        "method_revision": METHOD_SHA,
        "technical_base": BASE_SHA,
        "primary_domain": "observed_unknown AND predicted_free_j",
        "prediction_threshold": "<0.5",
        "members": ["G1","G2","G3"],
        "primary_metrics": {
            "R_MAP_Aj_m2": "count((raw<0) AND (Gj_runtime<0.5))*resolution^2",
            "R_MAP_A_mean_m2": "mean(A1,A2,A3)",
            "KnownFree_map_m2": "count(raw==0)*resolution^2",
            "MapRemainingFraction": "A_map_mean/(KnownFree_map + A_map_mean)",
        },
        "paired_orientation": {
            "A_signed_difference_m2": "map - reachable",
            "A_ratio_map_over_reachable": "map / reachable",
            "A_zero_denominator_reason": "ZERO_REACHABLE_AREA_DENOMINATOR",
            "Fraction_signed_difference": "map - reachable",
            "Fraction_ratio_map_over_reachable": "map / reachable",
            "Fraction_zero_denominator_reason": "ZERO_REACHABLE_FRACTION_DENOMINATOR",
        },
        "subset_invariant": "for each member on paired valid decisions: A_map_j_m2 >= A_reachable_j_m2",
        "oracle_role": "retrospective timing alignment only; not domain-matched map-only truth calibration",
        "secondary_comparator": "accepted D1/Gate-R reachable RemainingFraction/A_mean_m2 from shared_phase0_evidence_v1",
        "forbidden": ["reachability in primary metric","hybrid NA fill","resampling/reprojection","LaMa rerun","new simulation","retuning","online STOP","Gate-R relabel"],
    }
    (OUT / "MX025_R_MAP_METRIC_DICTIONARY.json").write_text(json.dumps(dictionary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    primary_pass = sum(int(r["R_MAP_evaluable"]) for r in per_decision)
    align_pass = sum(int(r["alignment_pass"]) for r in source_parity)
    reach_valid_all = sum(int(r["R_REACHABLE_evaluable"]) for r in per_decision)
    syn_index = {r["metric"]: r for r in synthesis if r["section"] == "PRIMARY_ORACLE_LOCAL"}
    miss_syn = [r for r in synthesis if r["section"] == "MISSINGNESS_RECOVERY"][0]
    pair_syn = [r for r in synthesis if r["section"] == "PAIRED_W10"][0]

    report = []
    report += ["# MX025 Analyst Report — Map-only R around Oracle-4", "",
               f"Accepted method revision: {METHOD_SHA}",
               f"Frozen technical base: {BASE_SHA}", "",
               "## A — Primary map-only R near Oracle-4"]
    for metric in ("MapRemainingFraction","R_MAP_A_mean_m2"):
        s = syn_index[metric]
        report.append(
            f"- {metric}: PRE→POST directions = {s['PRE_to_POST_decreasing']} decreasing, "
            f"{s['PRE_to_POST_increasing']} increasing, {s['PRE_to_POST_exact_zero']} exact-zero "
            f"across {s['runs_with_PRE_and_POST']} runs with both sides; run-macro median POST-minus-PRE = "
            f"{float(s['run_macro_median_POST_minus_PRE']):.9g}."
        )
    report += ["- Oracle-4 is used only as a timing anchor. MapRemainingFraction is not treated as true remaining fraction or directly calibrated to Oracle-4.", "",
               "## B — Reachable-R comparison and missingness recovery",
               f"- Primary R-map is valid on {primary_pass}/365 decisions; exact raw/G1/G2/G3 alignment passes {align_pass}/365.",
               f"- Reachable-R comparator is valid on {reach_valid_all}/365 decisions.",
               f"- In W10, R-map is valid on {miss_syn['R_MAP_valid_rows']}/{miss_syn['W10_rows']} rows versus reachable-R {miss_syn['R_REACHABLE_valid_rows']}/{miss_syn['W10_rows']}; recovered map-only values where reachable-R is NA = {miss_syn['R_MAP_valid_R_REACHABLE_NA_recovered_rows']}.",
               f"- W10 paired rows = {pair_syn['paired_rows']}; median area difference map-minus-reachable = {float(pair_syn['A_signed_difference_m2_median']):.9g} m²; median area ratio map/reachable = {float(pair_syn['A_ratio_map_over_reachable_median']):.9g}.",
               f"- W10 median normalized-fraction difference map-minus-reachable = {float(pair_syn['Fraction_signed_difference_median']):.9g}; median fraction ratio map/reachable = {float(pair_syn['Fraction_ratio_map_over_reachable_median']):.9g}.",
               "- The area subset invariant A_map_j >= A_reachable_j passed for every paired valid decision/member.", "",
               "## C — Exact mpx_004 W10 sanity table",
               "",
               "| decision | band | delta_p | A_map_mean m² | KnownFree_map m² | MapRemainingFraction | reachable valid | A_reachable_mean m² | ReachableFraction | reachable reason |",
               "|---:|---|---:|---:|---:|---:|---:|---:|---:|---|"]
    for r in m4_table:
        def fnum(v, digits=6):
            x = finite(v)
            return "NA" if not math.isfinite(x) else f"{x:.{digits}f}"
        report.append(
            f"| {r['decision_id']} | {r['band']} | {fnum(r['delta_p'],6)} | {fnum(r['R_MAP_A_mean_m2'],6)} | "
            f"{fnum(r['KnownFree_map_m2'],6)} | {fnum(r['MapRemainingFraction'],6)} | {r['R_REACHABLE_evaluable']} | "
            f"{fnum(r['R_REACHABLE_A_mean_m2'],6)} | {fnum(r['R_REACHABLE_RemainingFraction'],6)} | {r['R_REACHABLE_reason'] or ''} |"
        )
    report += ["", "## D — Interpretation boundary",
               "- Removing reachability increases data availability and measures a different map-level quantity; this does not show that reachability is unnecessary for planning.",
               "- Reachable-R remains a separate accepted historical metric. MX025 does not reopen or relabel Gate R.",
               "- No LaMa/model rerun, simulation, prediction regeneration, interpolation, retuning, threshold change, composite, or online STOP rule was used.",
               "- Scope is the frozen New Room development cohort around retrospective Oracle-4 only."]
    (OUT / "MX025_ANALYST_REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    artifacts = [
        OUT / "MX025_R_MAP_PER_DECISION.csv",
        OUT / "MX025_R_MAP_PER_RUN.csv",
        OUT / "MX025_R_MAP_ORACLE_SYNTHESIS.csv",
        OUT / "MX025_R_MAP_VS_REACHABLE_PAIRED.csv",
        OUT / "MX025_R_MAP_MISSINGNESS_RECOVERY.csv",
        OUT / "MX025_R_MAP_MPX004_W10_SANITY.csv",
        OUT / "MX025_R_MAP_METRIC_DICTIONARY.json",
        OUT / "MX025_R_MAP_SOURCE_PARITY.csv",
        OUT / "MX025_ANALYST_REPORT.md",
    ]
    manifest = {
        "schema": "mx025_map_only_r_execution_v1",
        "status": "COMPLETE_PENDING_IR2_RESULT_QA",
        "method_revision": METHOD_SHA,
        "technical_base": BASE_SHA,
        "cohort": {"runs": RUNS, "decisions": 365, "oracle_anchors": ORACLE, "W10_rows": len(w10_total)},
        "integrity": {
            "raw_g1_g2_g3_alignment_pass": align_pass,
            "primary_r_map_evaluable": primary_pass,
            "reachable_r_evaluable_all": reach_valid_all,
            "paired_all_rows": len(paired),
            "W10_r_map_valid": len(w10_map_valid),
            "W10_reachable_valid": len(w10_reach_valid),
            "W10_recovered_reachable_NA": len(recovery),
            "W10_paired_rows": len(w10_paired),
            "subset_invariant_violations": 0,
            "mpx_004_W10_rows": len(m4_table),
            "mpx_004_W10_reachable_NA_rows": sum(1 for r in m4_table if not int(r["R_REACHABLE_evaluable"])),
        },
        "guards": {
            "model_rerun": False, "simulation_rerun": False, "prediction_regeneration": False,
            "reachability_in_primary": False, "hybrid_NA_fill": False, "retuning": False,
            "online_stop": False, "gate_r_relabel": False,
        },
        "artifacts": [{"path": str(p.relative_to(OUT)), "sha256": digest(p), "size": p.stat().st_size} for p in artifacts],
    }
    (OUT / "MX025_R_MAP_ARTIFACT_MANIFEST.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"integrity": manifest["integrity"], "synthesis": synthesis}, indent=2, sort_keys=True))

if __name__ == "__main__":
    main()
