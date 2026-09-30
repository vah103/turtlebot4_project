#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, json, math
from collections import Counter, defaultdict
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from mapex_lab.analysis.d1 import d1_gate_p as gate

METHOD_SHA="f5144e2727993b920d172fa106ad5477a9c6fbe6"
BASE_SHA="4899ee95c85640965befeaf98c20f156c0d3181a"
P_RESULT_SHA="9998225b66a47962c91526a1ce39b0a37f03950b"
U_RESULT_SHA="d9dd15a872d57cd63ff109abae01c581dd582f55"
R_RESULT_SHA="c9b0b3fad47839a4e0ed6c29cb4b17d50965b78e"
LAB=Path(__file__).resolve().parents[1]
EXP=LAB/"experiments"/"mapex"
GT_PATH=LAB/"ground_truth"/"new_room"/"generated"/"new_room_structural_gt_v2.npz"
GATE_P=LAB/"analysis"/"d1"/"results"/"gate_p"/"gate_p_decisions.csv"
SHARED=LAB/"analysis"/"d1"/"results"/"shared_phase0_evidence_v1"/"shared_phase0_evidence.csv"
U_DEC=LAB/"analysis"/"mx026_inputs"/"MX024_U_PER_DECISION_ACCEPTED.csv"
U_ERR=LAB/"analysis"/"mx026_inputs"/"MX024_U_ERROR_ALIGNMENT_PER_DECISION_ACCEPTED.csv"
R_DEC=LAB/"analysis"/"mx026_inputs"/"MX025_R_MAP_PER_DECISION_ACCEPTED.csv"
OUT=LAB/"analysis"/"mx026_exec_results"; FIG=OUT/"figures"
ORACLE={"mpx_001":21,"mpx_002":18,"mpx_003":21,"mpx_004":16,"mpx_005":16,"mpx_006":20,"mpx_007":18,"mpx_008":19,"mpx_009":18,"mpx_010":19}
RUNS=list(ORACLE)
PMS=["FreeRecall","FreePrecision","MissedFreeRate","FreeIoU","OccupiedRecall","OccupiedPrecision","FalseOpenRate","OccupiedIoU","BalancedRecall","MacroIoU"]
WHOLE=[("P_STRUCT_FreeRecall","Q1_STRUCTURAL_GT"),("P_STRUCT_FreePrecision","Q1_STRUCTURAL_GT"),("P_STRUCT_MissedFreeRate","Q1_STRUCTURAL_GT"),("P_STRUCT_OccupiedRecall","Q1_STRUCTURAL_GT"),("P_STRUCT_FalseOpenRate","Q1_STRUCTURAL_GT"),("U_MAPEX_U_mean","MAPEX_UNKNOWN_VALID"),("U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID"),("R_MapRemainingFraction","R_MAP_ONLY"),("R_A_map_mean_m2","R_MAP_ONLY")]
BINMET=[*(("P_STRUCT_"+m,"Q1_STRUCTURAL_GT") for m in PMS),("U_MAPEX_U_mean","MAPEX_UNKNOWN_VALID"),("U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID"),("U_MAPEX_U_disagreement","MAPEX_UNKNOWN_VALID"),("R_MapRemainingFraction","R_MAP_ONLY"),("R_A_map_mean_m2","R_MAP_ONLY")]
PAIRS=[("U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID","P_STRUCT_MissedFreeRate","Q1_STRUCTURAL_GT"),("U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID","P_STRUCT_FalseOpenRate","Q1_STRUCTURAL_GT"),("U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID","P_STRUCT_FreePrecision","Q1_STRUCTURAL_GT"),("R_MapRemainingFraction","R_MAP_ONLY","P_STRUCT_MissedFreeRate","Q1_STRUCTURAL_GT"),("R_MapRemainingFraction","R_MAP_ONLY","P_STRUCT_FalseOpenRate","Q1_STRUCTURAL_GT"),("R_MapRemainingFraction","R_MAP_ONLY","U_MAPEX_U_p95","MAPEX_UNKNOWN_VALID")]
BINS=[("B00_10",0,.1,0),("B10_20",.1,.2,0),("B20_30",.2,.3,0),("B30_40",.3,.4,0),("B40_50",.4,.5,0),("B50_60",.5,.6,0),("B60_70",.6,.7,0),("B70_80",.7,.8,0),("B80_90",.8,.9,0),("B90_100",.9,1,1)]

def read_csv(p):
    with Path(p).open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rows):
    rows=list(rows); Path(p).parent.mkdir(parents=True,exist_ok=True); fields=[]
    for r in rows:
        for k in r:
            if k not in fields:fields.append(k)
    with Path(p).open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def digest(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()
def fin(v):
    try:x=float(v)
    except:return math.nan
    return x if math.isfinite(x) else math.nan
def bval(v):return str(v).strip().lower() in {"1","true","yes"}
def div(a,b):return float(a/b) if b else math.nan
def q(vals,p):
    a=np.asarray([float(x) for x in vals if math.isfinite(float(x))],float)
    return float(np.quantile(a,p,method="linear")) if a.size else math.nan
def med(vals):return q(vals,.5)

def pm(FF,FO,OF,OO):
    fr=div(FF,FF+FO); op=div(OO,OO+OF)
    fi=div(FF,FF+FO+OF); oi=div(OO,OO+OF+FO)
    return {"FreeRecall":fr,"FreePrecision":div(FF,FF+OF),"MissedFreeRate":div(FO,FF+FO),"FreeIoU":fi,
            "OccupiedRecall":op,"OccupiedPrecision":div(OO,OO+FO),"FalseOpenRate":div(OF,OO+OF),"OccupiedIoU":oi,
            "BalancedRecall":float(np.nanmean([fr,op])),"MacroIoU":float(np.nanmean([fi,oi]))}

def pfrom(row):
    FF=int(float(row["mean_tn_free"])); FO=int(float(row["mean_fp_occ"])); OF=int(float(row["mean_fn_occ"])); OO=int(float(row["mean_tp_occ"]))
    ca=div(float(row["evaluated_area_m2"]),int(row["evaluated_cell_count"]))
    out={"FF":FF,"FO":FO,"OF":OF,"OO":OO,"TF":FF+FO,"TO":OF+OO,"PF":FF+OF,"PO":FO+OO,
         "true_free_count":FF+FO,"true_occupied_count":OF+OO,"predicted_free_count":FF+OF,"predicted_occupied_count":FO+OO,
         "true_free_area_m2":(FF+FO)*ca,"true_occupied_area_m2":(OF+OO)*ca,
         "predicted_free_area_m2":(FF+OF)*ca,"predicted_occupied_area_m2":(FO+OO)*ca,"scoreable_count":int(row["evaluated_cell_count"])}
    out.update(pm(FF,FO,OF,OO))
    chk={"FreeRecall":"mean_free_recall","FreePrecision":"mean_free_precision","FreeIoU":"mean_free_iou","OccupiedRecall":"mean_occupied_recall","OccupiedPrecision":"mean_occupied_precision","OccupiedIoU":"mean_occupied_iou","MacroIoU":"mean_macro_iou"}
    for k,s in chk.items():
        a=fin(out[k]);b=fin(row[s])
        if math.isfinite(a)!=math.isfinite(b) or (math.isfinite(a) and abs(a-b)>1e-12):raise RuntimeError("GATE_P_PARITY_FAIL:"+row["run_id"]+":"+row["decision_id"]+":"+row["reference"]+":"+k)
    return out

def classm(vals,truth,leq):
    vals=np.asarray(vals,float); truth=np.asarray(truth,bool)
    pf=vals<=.5 if leq else vals<.5; po=~pf; tf=~truth
    FF=int(np.sum(pf&tf));FO=int(np.sum(po&tf));OF=int(np.sum(pf&truth));OO=int(np.sum(po&truth))
    o={"FF":FF,"FO":FO,"OF":OF,"OO":OO};o.update(pm(FF,FO,OF,OO));return o

def ranks(v):
    a=np.asarray(v,float);order=np.argsort(a,kind="mergesort");r=np.empty(len(a),float);i=0
    while i<len(a):
        j=i+1
        while j<len(a) and a[order[j]]==a[order[i]]:j+=1
        r[order[i:j]]=((i+1)+j)/2.;i=j
    return r

def rho_row(run,xm,xd,ym,yd,rows):
    cand=len(rows); pairs=[(fin(r.get(xm)),fin(r.get(ym))) for r in rows];pairs=[z for z in pairs if math.isfinite(z[0]) and math.isfinite(z[1])];n=len(pairs)
    o={"run_id":run,"x_metric":xm,"x_domain":xd,"y_metric":ym,"y_domain":yd,"candidate_row_n":cand,"excluded_nonfinite_n":cand-n,"paired_finite_n":n,"rho":math.nan,"rho_reason":""}
    if n<5:o["rho_reason"]="INSUFFICIENT_FINITE_PAIRED_SUPPORT_LT5";return o
    x=np.asarray([z[0] for z in pairs]);y=np.asarray([z[1] for z in pairs]);xc=np.all(x==x[0]);yc=np.all(y==y[0])
    if xc and yc:o["rho_reason"]="CONSTANT_BOTH_VECTORS";return o
    if xc:o["rho_reason"]="CONSTANT_X_VECTOR";return o
    if yc:o["rho_reason"]="CONSTANT_Y_VECTOR";return o
    o["rho"]=float(np.corrcoef(ranks(x),ranks(y))[0,1]);return o

def pbin(p):
    for n,lo,hi,last in BINS:
        if (lo<=p<=hi) if last else (lo<=p<hi):return n
    raise RuntimeError("BIN_FAIL:"+str(p))

def pref(dst,p,src):
    for k,v in src.items():dst[p+k]=v

def plot_run(rid,rows):
    x=np.asarray([float(r["normalized_progress"]) for r in rows]); ox=float([r["normalized_progress"] for r in rows if int(r["Oracle4_marker"])][0])
    fig,ax=plt.subplots(5,1,figsize=(12,17),sharex=True)
    for m in ("FreeRecall","FreePrecision","MissedFreeRate","OccupiedRecall","FalseOpenRate"):ax[0].plot(x,[fin(r.get("P_STRUCT_"+m)) for r in rows],label=m)
    ax[0].axvline(ox,ls="--");ax[0].set_ylabel("P structural");ax[0].legend(ncol=3,fontsize=7);ax[0].set_title("MX026 "+rid+" full trajectory")
    ax[1].plot(x,[fin(r.get("P_STRUCT_true_free_count")) for r in rows],label="true_free_count");ax[1].plot(x,[fin(r.get("P_STRUCT_true_occupied_count")) for r in rows],label="true_occupied_count")
    ax[1].axvline(ox,ls="--");ax[1].set_ylabel("P support");ax[1].legend(fontsize=7)
    for m in ("FreeRecall","FreePrecision","MissedFreeRate","OccupiedRecall","FalseOpenRate"):ax[2].plot(x,[fin(r.get("P_LATER_"+m)) for r in rows],label=m)
    ax[2].axvline(ox,ls="--");ax[2].set_ylabel("P later");ax[2].legend(ncol=3,fontsize=7)
    for m in ("U_MAPEX_U_mean","U_MAPEX_U_p95","U_MAPEX_U_disagreement"):ax[3].plot(x,[fin(r.get(m)) for r in rows],label=m.replace("U_MAPEX_",""))
    ax[3].axvline(ox,ls="--");ax[3].set_ylabel("MapEx U");ax[3].legend(fontsize=7)
    b=ax[4].twinx();ax[4].plot(x,[fin(r.get("R_MapRemainingFraction")) for r in rows],label="MapRemainingFraction");b.plot(x,[fin(r.get("R_A_map_mean_m2")) for r in rows],label="A_map_mean_m2")
    ax[4].plot(x,[fin(r.get("R_REACHABLE_RemainingFraction")) for r in rows],label="reachable fraction",ls=":");ax[4].axvline(ox,ls="--");ax[4].set_ylabel("R fraction");b.set_ylabel("R area m2")
    h1,l1=ax[4].get_legend_handles_labels();h2,l2=b.get_legend_handles_labels();ax[4].legend(h1+h2,l1+l2,fontsize=7);ax[4].set_xlabel("normalized progress")
    fig.tight_layout();p=FIG/f"MX026_{rid}_FULL_TRAJECTORY.svg";fig.savefig(p);plt.close(fig);return p

def main():
    OUT.mkdir(parents=True,exist_ok=True);FIG.mkdir(parents=True,exist_ok=True)
    gp=read_csv(GATE_P);shrows=read_csv(SHARED);ur=read_csv(U_DEC);ue=read_csv(U_ERR);rr=read_csv(R_DEC)
    gs={};gl={}
    for r in gp:
        k=(r["run_id"],int(r["decision_id"]))
        if r["reference"]=="structural_gt":gs[k]=r
        elif r["reference"]=="later_observed":gl[k]=r
    if len(gs)!=365 or len(gl)!=365:raise RuntimeError("P_INVENTORY_FAIL:"+str((len(gs),len(gl))))
    sh={(r["run_id"],int(r["decision_id"])):r for r in shrows};u={(r["run_id"],int(r["decision_id"])):r for r in ur};rm={(r["run_id"],int(r["decision_id"])):r for r in rr}
    if not(len(sh)==len(u)==len(rm)==365):raise RuntimeError("PRIMARY_INPUT_FAIL")
    uerr={(r["run_id"],int(r["decision_id"]),r["domain"]):r for r in ue}
    if len(uerr)!=730:raise RuntimeError("UERR_FAIL")
    gt=gate.load_structural_gt(GT_PATH);master=[];bound=[];pparity=[]
    for rid in RUNS:
        run=EXP/rid;table=sorted(gate._read_csv(run/"decisions.csv"),key=lambda r:int(r["decision_id"]));grids=[gate.load_raw_grid(gate._resolve_run_path(run,r["raw_map"])) for r in table];N=len(table)
        for idx,d in enumerate(table,1):
            did=int(d["decision_id"]);k=(rid,did);prog=(idx-1)/(N-1)
            if k not in gs or k not in sh or k not in u or k not in rm:raise RuntimeError("MISSING:"+str(k))
            if abs(prog-float(u[k]["progress"]))>1e-12 or abs(prog-float(rm[k]["progress"]))>1e-12:raise RuntimeError("PROGRESS_MISMATCH:"+str(k))
            o={"run_id":rid,"decision_id":did,"decision_index":idx,"decision_count":N,"normalized_progress":prog,"progress_bin":pbin(prog),"Oracle4_marker":int(did==ORACLE[rid])}
            ps=pfrom(gs[k]);o["P_structural_evaluable"]=1;o["P_structural_reason"]="";pref(o,"P_STRUCT_",ps);pparity.append({"run_id":rid,"decision_id":did,"domain":"Q1_STRUCTURAL_GT","parity_pass":1})
            pl=pfrom(gl[k]);later_ok=int(pl["scoreable_count"]>0);o["P_later_observed_evaluable"]=later_ok;o["P_later_observed_reason"]="" if later_ok else "NO_LATER_OBSERVED_TARGETS";pref(o,"P_LATER_",pl);pparity.append({"run_id":rid,"decision_id":did,"domain":"Q1_LATER_OBSERVED","parity_pass":1})
            s=sh[k];te=bval(s.get("primary_topology_risk_evaluable",""));o["P_topology_evaluable"]=int(te);o["P_topology_reason"]="" if te else (s.get("r004_topology_source_reason","") or s.get("non_evaluable_reasons","") or "TOPOLOGY_NOT_EVALUABLE")
            tmap={"NavigableMissedFreeFraction":"r004_topology_navigable_missed_free_fraction","ReachableFutureFreeRetention":"r004_topology_reachable_future_free_retention","ReachablePairConnectivityRetention":"r004_topology_reachable_pair_connectivity_retention","LargestPredictedPieceFraction":"r004_topology_largest_predicted_piece_fraction","source_prediction_traversable":"r004_topology_source_prediction_traversable","fragmented":"r004_topology_fragmented"}
            for a,b in tmap.items():o["P_TOPO_"+a]=s.get(b,"")
            z=u[k];o["U_evaluable"]=int(str(z.get("primary_Q1_evaluable",""))=="1");o["U_reason"]=z.get("reason","")
            for a,b in [("MAPEX_U_mean","MAPEX_U_mean"),("MAPEX_U_p95","MAPEX_U_p95"),("MAPEX_U_disagreement","MAPEX_U_disagreement"),("MAPEX_UNKNOWN_VALID_count","support_count")]:o["U_"+b]=z.get(a,"")
            for dom,p in [("STRUCTURAL_GT","U_ERR_STRUCT_"),("LATER_OBSERVED","U_ERR_LATER_")]:
                e=uerr[(rid,did,dom)];o[p+"evaluable"]=e.get("evaluable","0");o[p+"reason"]=e.get("reason","")
                for m in ("HighUErrorRate","BaselineErrorRate","ErrorEnrichment95","ErrorCapture95","MissedFreeCapture95","FalseOpenCapture95"):o[p+m]=e.get(m,"")
            z=rm[k];o["R_map_evaluable"]=z.get("R_MAP_evaluable","0");o["R_map_reason"]=z.get("R_MAP_reason","")
            for a,b in [("R_MAP_A1_m2","A_map_1_m2"),("R_MAP_A2_m2","A_map_2_m2"),("R_MAP_A3_m2","A_map_3_m2"),("R_MAP_A_mean_m2","A_map_mean_m2"),("R_MAP_A_min_m2","A_map_min_m2"),("R_MAP_A_max_m2","A_map_max_m2"),("R_MAP_A_range_m2","A_map_range_m2"),("KnownFree_map_m2","KnownFree_map_m2"),("MapRemainingFraction","MapRemainingFraction")]:o["R_"+b]=z.get(a,"")
            o["R_reachable_evaluable"]=z.get("R_REACHABLE_evaluable","0");o["R_reachable_reason"]=z.get("R_REACHABLE_reason","");o["R_REACHABLE_A_mean_m2"]=z.get("R_REACHABLE_A_mean_m2","");o["R_REACHABLE_RemainingFraction"]=z.get("R_REACHABLE_RemainingFraction","");o["R_REACHABLE_KnownReachableFree_m2"]=z.get("R_REACHABLE_KnownReachableFree_m2","")
            obs=grids[idx-1];pred=gate.load_runtime_prediction(gate._resolve_run_path(run,d["mean_map"]),obs,"mean")
            c=gate._structural_context(obs,gt);sv=gate._structural_prediction_values(pred,c);st=np.asarray(c["truth_occupied"],bool)
            r0,c0=np.nonzero(obs.data<0);lab,_,_,_=gate.first_later_observation_targets(obs,r0,c0,idx,table,grids);keep=lab>=0;lv=pred[r0[keep],c0[keep]];lt=lab[keep]>0;gh=int(np.count_nonzero(pred==.5))
            for dom,vals,truth in [("Q1_STRUCTURAL_GT",sv,st),("Q1_LATER_OBSERVED",lv,lt)]:
                half=int(np.count_nonzero(vals==.5));primary=classm(vals,truth,False);rec={"run_id":rid,"decision_id":did,"domain":dom,"scoreable_cell_count":int(vals.size),"global_exact_p_eq_0p5_count":gh,"exact_p_eq_0p5_count":half,"boundary_sensitivity_required":int(half>0)}
                for a,v in primary.items():rec["primary_"+a]=v
                if half:
                    alt=classm(vals,truth,True)
                    for a,v in alt.items():
                        rec["alternate_"+a]=v
                        rec["delta_"+a]=(int(v)-int(primary[a])) if a in ("FF","FO","OF","OO") else ((fin(v)-fin(primary[a])) if math.isfinite(fin(v)) and math.isfinite(fin(primary[a])) else math.nan)
                bound.append(rec)
            master.append(o)
    if len(master)!=365 or len(bound)!=730:raise RuntimeError("OUTPUT_MEMBERSHIP_FAIL")
    bi={(r["run_id"],int(r["decision_id"]),r["domain"]):r for r in bound}
    for src,dom in [(gs,"Q1_STRUCTURAL_GT"),(gl,"Q1_LATER_OBSERVED")]:
        for k,g in src.items():
            b=bi[(k[0],k[1],dom)]
            for bk,gk in [("primary_FF","mean_tn_free"),("primary_FO","mean_fp_occ"),("primary_OF","mean_fn_occ"),("primary_OO","mean_tp_occ")]:
                if int(b[bk])!=int(float(g[gk])):raise RuntimeError("BOUNDARY_PRIMARY_PARITY_FAIL:"+str(k))
    write_csv(OUT/"MX026_PUR_PER_DECISION.csv",master);write_csv(OUT/"MX026_P_SOURCE_PARITY.csv",pparity);write_csv(OUT/"MX026_P_BOUNDARY_AUDIT.csv",bound)
    byr={rid:[r for r in master if r["run_id"]==rid] for rid in RUNS}
    prun=[];urun=[];rrun=[]
    for rid,rows in byr.items():
        a={"run_id":rid,"decision_count":len(rows),"P_structural_valid_n":sum(int(r["P_structural_evaluable"]) for r in rows),"P_later_observed_valid_n":sum(int(r["P_later_observed_evaluable"]) for r in rows),"P_topology_valid_n":sum(int(r["P_topology_evaluable"]) for r in rows)}
        for p in ("P_STRUCT_","P_LATER_"):
            for m in PMS:a[p+m+"_median"]=med([fin(r.get(p+m)) for r in rows if math.isfinite(fin(r.get(p+m)))])
        prun.append(a)
        a={"run_id":rid,"decision_count":len(rows),"U_valid_n":sum(int(r["U_evaluable"]) for r in rows)}
        for m in ("U_MAPEX_U_mean","U_MAPEX_U_p95","U_MAPEX_U_disagreement"):
            v=[fin(r.get(m)) for r in rows if math.isfinite(fin(r.get(m)))];a[m+"_first"]=v[0] if v else math.nan;a[m+"_final"]=v[-1] if v else math.nan;a[m+"_median"]=med(v)
        urun.append(a)
        a={"run_id":rid,"decision_count":len(rows),"R_map_valid_n":sum(str(r["R_map_evaluable"])=="1" for r in rows),"R_reachable_valid_n":sum(str(r["R_reachable_evaluable"])=="1" for r in rows)}
        for m in ("R_MapRemainingFraction","R_A_map_mean_m2"):
            v=[fin(r.get(m)) for r in rows if math.isfinite(fin(r.get(m)))];a[m+"_first"]=v[0] if v else math.nan;a[m+"_final"]=v[-1] if v else math.nan;a[m+"_median"]=med(v)
        rrun.append(a)
    write_csv(OUT/"MX026_P_PER_RUN.csv",prun);write_csv(OUT/"MX026_U_PER_RUN.csv",urun);write_csv(OUT/"MX026_R_PER_RUN.csv",rrun)
    brows=[];cache={}
    for rid,rows in byr.items():
        for metric,dom in BINMET:
            for bn,_,_,_ in BINS:
                v=[fin(r.get(metric)) for r in rows if r["progress_bin"]==bn and math.isfinite(fin(r.get(metric)))]
                rec={"row_type":"RUN_BIN","run_id":rid,"metric":metric,"domain":dom,"progress_bin":bn,"valid_decision_n":len(v),"support":len(v),"run_bin_median":med(v)};brows.append(rec);cache[(rid,metric,bn)]=rec
    for metric,dom in BINMET:
        for bn,_,_,_ in BINS:
            v=[fin(cache[(rid,metric,bn)]["run_bin_median"]) for rid in RUNS if math.isfinite(fin(cache[(rid,metric,bn)]["run_bin_median"]))]
            brows.append({"row_type":"COHORT_BIN","run_id":"","metric":metric,"domain":dom,"progress_bin":bn,"run_contributor_n":len(v),"run_macro_median":med(v),"run_macro_q25":q(v,.25),"run_macro_q75":q(v,.75)})
    write_csv(OUT/"MX026_PUR_PROGRESS_BINS.csv",brows)
    whole=[]
    for rid,rows in byr.items():
        for m,d in WHOLE:
            vv=[fin(r.get(m)) for r in rows];valid=[x for x in vv if math.isfinite(x)];rec=rho_row(rid,"normalized_progress","NORMALIZED_PROGRESS",m,d,rows);first=valid[0] if valid else math.nan;last=valid[-1] if valid else math.nan
            rec.update({"metric":m,"metric_domain":d,"first_valid_value":first,"final_valid_value":last,"final_minus_first":last-first if math.isfinite(first) and math.isfinite(last) else math.nan,"valid_n":len(valid),"total_n":len(rows)});whole.append(rec)
    write_csv(OUT/"MX026_PUR_WHOLE_RUN_DIRECTION.csv",whole)
    pairs=[]
    for rid,rows in byr.items():
        for xm,xd,ym,yd in PAIRS:pairs.append(rho_row(rid,xm,xd,ym,yd,rows))
    if len(pairs)!=60:raise RuntimeError("PAIR_UNIVERSE_FAIL")
    write_csv(OUT/"MX026_PUR_PAIRWISE_CONTEXT.csv",pairs)
    bsum=[]
    for dom in ("Q1_STRUCTURAL_GT","Q1_LATER_OBSERVED"):
        rs=[r for r in bound if r["domain"]==dom];aff=[r for r in rs if int(r["exact_p_eq_0p5_count"])>0]
        bsum.append({"domain":dom,"decision_rows":len(rs),"scoreable_rows":sum(int(r["scoreable_cell_count"])>0 for r in rs),"total_scoreable_cells":sum(int(r["scoreable_cell_count"]) for r in rs),"rows_with_exact_p_eq_0p5":len(aff),"total_exact_p_eq_0p5_cells":sum(int(r["exact_p_eq_0p5_count"]) for r in rs),"affected_identities":";".join(r["run_id"]+":d"+str(int(r["decision_id"])) for r in aff)})
    (OUT/"MX026_P_BOUNDARY_AUDIT_SUMMARY.json").write_text(json.dumps(bsum,indent=2,sort_keys=True)+"\n")
    plots=[plot_run(r,byr[r]) for r in RUNS]
    widx=defaultdict(list)
    for r in whole:widx[r["metric"]].append(r)
    pidx=defaultdict(list)
    for r in pairs:pidx[(r["x_metric"],r["y_metric"])].append(r)
    dictionary={"task":"MX026","method_revision":METHOD_SHA,"technical_base":BASE_SHA,
      "accepted_sources":{"P_result":P_RESULT_SHA,"U_result":U_RESULT_SHA,"R_result":R_RESULT_SHA,"gate_p_blob":"d3826921e0540e05ed1de40bed2d58c76e2fb90d","shared_phase0_blob":"3e80124d545ea37b2791594dd775a7d953368bea","U_input_blob":"4bbdf60198a1df87e8d34e69f7b08221e0f9532b","U_error_input_blob":"b10e8e09b86b27f8a2faebaa1cf1d785433cfe57","R_input_blob":"f9a79d8f4edb7453832e4b828136a65009712260"},
      "primary_scope":"all 365 decisions; Oracle-4 marker only","P_threshold":"free iff p<0.5; occupied iff p>=0.5","boundary_audit":"exact p==0.5 only; alternate free iff p<=0.5 only on affected row/domain","U_semantics":"MX024 original MapEx variance n=3 ddof=1","R_semantics":"MX025 map-only","spearman":{"minimum_n":5,"ties":"average ranks","p_values":False},"mandatory_pairs":[{"x":a,"x_domain":b,"y":c,"y_domain":d} for a,b,c,d in PAIRS],"forbidden":["cross-fill NA","interpolation","smoothing","composite","winner","retuning","online STOP","causal claim"]}
    (OUT/"MX026_PUR_METRIC_DICTIONARY.json").write_text(json.dumps(dictionary,indent=2,sort_keys=True)+"\n")
    lines=["# MX026 Analyst Report — full-trajectory P/U/R survey","",
      "Accepted methodology: "+METHOD_SHA,"Frozen technical base: "+BASE_SHA,"",
      "## 1. Inventory and evaluability",
      "- Master trajectory: 365/365 unique decisions.",
      "- P structural: "+str(sum(int(r["P_structural_evaluable"]) for r in master))+"/365; P later-observed: "+str(sum(int(r["P_later_observed_evaluable"]) for r in master))+"/365; P topology: "+str(sum(int(r["P_topology_evaluable"]) for r in master))+"/365.",
      "- U primary: "+str(sum(int(r["U_evaluable"]) for r in master))+"/365.",
      "- R map-only: "+str(sum(str(r["R_map_evaluable"])=="1" for r in master))+"/365; reachable-R secondary: "+str(sum(str(r["R_reachable_evaluable"])=="1" for r in master))+"/365.","","## 2. Whole-run direction summaries"]
    for m,d in WHOLE:
        rs=widx[m];dv=[fin(r["final_minus_first"]) for r in rs if math.isfinite(fin(r["final_minus_first"]))];rh=[fin(r["rho"]) for r in rs if math.isfinite(fin(r["rho"]))]
        lines.append("- "+m+": final-minus-first "+str(sum(x<0 for x in dv))+" negative / "+str(sum(x>0 for x in dv))+" positive / "+str(sum(x==0 for x in dv))+" zero; run-macro median delta="+format(med(dv),".9g")+"; median valid rho(progress)="+format(med(rh),".6g")+" (n="+str(len(rh))+").")
    lines+=["","## 3. Six frozen cross-family associations"]
    for xm,xd,ym,yd in PAIRS:
        rs=pidx[(xm,ym)];v=[fin(r["rho"]) for r in rs if math.isfinite(fin(r["rho"]))];reasons=Counter(r["rho_reason"] for r in rs if r["rho_reason"])
        lines.append("- "+xm+" vs "+ym+": valid rho "+str(len(v))+"/10; run-macro median rho="+format(med(v),".6g")+"; negative/positive/zero="+str(sum(x<0 for x in v))+"/"+str(sum(x>0 for x in v))+"/"+str(sum(x==0 for x in v))+"; NA reasons="+str(dict(reasons))+".")
    lines+=["","## 4. Exact p=0.5 boundary audit"]
    for b in bsum:lines.append("- "+b["domain"]+": scoreable rows "+str(b["scoreable_rows"])+"/365; exact-boundary rows "+str(b["rows_with_exact_p_eq_0p5"])+"; exact-boundary cells "+str(b["total_exact_p_eq_0p5_cells"])+"; affected identities: "+(b["affected_identities"] or "none")+".")
    lines+=["","## 5. Interpretation boundary","- Full 365-decision trajectory is primary; fixed progress bins are secondary equal-run summaries.","- Oracle-4 is marker only and did not select rows, bins, metrics, pairs, or conclusions.","- Family-specific missingness remains separate; no cross-fill, interpolation or smoothing.","- Six pairwise rhos are descriptive only; no p-values, significance labels, composite, winner, causal claim, retuning or online STOP."]
    (OUT/"MX026_ANALYST_REPORT.md").write_text("\n".join(lines)+"\n")
    arts=[OUT/"MX026_PUR_PER_DECISION.csv",OUT/"MX026_P_PER_RUN.csv",OUT/"MX026_U_PER_RUN.csv",OUT/"MX026_R_PER_RUN.csv",OUT/"MX026_PUR_PROGRESS_BINS.csv",OUT/"MX026_PUR_WHOLE_RUN_DIRECTION.csv",OUT/"MX026_PUR_PAIRWISE_CONTEXT.csv",OUT/"MX026_P_SOURCE_PARITY.csv",OUT/"MX026_P_BOUNDARY_AUDIT.csv",OUT/"MX026_P_BOUNDARY_AUDIT_SUMMARY.json",OUT/"MX026_PUR_METRIC_DICTIONARY.json",OUT/"MX026_ANALYST_REPORT.md",*plots]
    manifest={"schema":"mx026_full_trajectory_pur_execution_v1","status":"COMPLETE_PENDING_INDEPENDENT_RESULT_QA","method_revision":METHOD_SHA,"technical_base":BASE_SHA,"accepted_input_revisions":{"P":P_RESULT_SHA,"U":U_RESULT_SHA,"R":R_RESULT_SHA},"inventory":{"master_rows":len(master),"unique_master_keys":len({(r["run_id"],r["decision_id"]) for r in master}),"P_structural_evaluable":sum(int(r["P_structural_evaluable"]) for r in master),"P_later_observed_evaluable":sum(int(r["P_later_observed_evaluable"]) for r in master),"P_topology_evaluable":sum(int(r["P_topology_evaluable"]) for r in master),"U_evaluable":sum(int(r["U_evaluable"]) for r in master),"R_map_evaluable":sum(str(r["R_map_evaluable"])=="1" for r in master),"R_reachable_evaluable":sum(str(r["R_reachable_evaluable"])=="1" for r in master),"boundary_rows":len(bound),"mandatory_pair_rows":len(pairs),"whole_run_direction_rows":len(whole),"full_run_plot_count":len(plots)},"boundary_audit":bsum,"guards":{"all_365_primary":True,"oracle_marker_only":True,"cross_fill_NA":False,"interpolation":False,"smoothing":False,"composite":False,"winner":False,"retuning":False,"online_stop":False,"p_values":False},"artifacts":[{"path":str(p.relative_to(OUT)),"sha256":digest(p),"size":p.stat().st_size} for p in arts]}
    (OUT/"MX026_PUR_ARTIFACT_MANIFEST.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    print(json.dumps({"inventory":manifest["inventory"],"boundary_audit":bsum},indent=2,sort_keys=True))
if __name__=="__main__":main()
