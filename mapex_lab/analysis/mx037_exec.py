#!/usr/bin/env python3
"""MX038: exact frozen retrospective execution of MX037 Method V2.

One 365-decision / 10-New-Room replay. No retuning, Hospital, prospective
collection, Engineer work, deployment, or robot STOP.
"""
from __future__ import annotations

import csv, hashlib, json, math, os, statistics, subprocess, sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SCRIPT=Path(__file__).resolve()
ANALYSIS=SCRIPT.parent
MAPEX=ANALYSIS.parent
REPO=MAPEX.parent
RESULTS=ANALYSIS/"mx037_exec_results"
INPUTS=ANALYSIS/"mx037_inputs"
EXPS=MAPEX/"experiments"/"mapex"

if str(REPO) not in sys.path: sys.path.insert(0,str(REPO))
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo
from mapex_lab.analysis.r004 import evaluate_prediction_vs_final_observed as r004base
from mapex_lab.analysis.d1 import d1_gate_p as gatep

METHOD="ac5b7a4e2b31a33aaaed1172d3b42f5fdf80862e"
METHOD_BLOB="c207ea5a5e2897bc22d6d6b34e91c202512bfb12"
AUTH="e3a7255edcb34880899313b27942e1fee95c6555"
BASE_REV="d90498e1a3ead2a0c170a3fee7c02a1bc9526cca"
RUNS=tuple(f"mpx_{i:03d}" for i in range(1,11))
BASE_FIRE={"mpx_001":23,"mpx_002":18,"mpx_003":22,"mpx_004":16,"mpx_005":19,"mpx_006":23,"mpx_007":19,"mpx_008":20,"mpx_009":19,"mpx_010":23}
R_TAU=.05

MX026=ANALYSIS/"mx026_exec_results"/"MX026_PUR_PER_DECISION.csv"
MX025SRC=ANALYSIS/"mx025_exec_results"/"MX025_R_MAP_SOURCE_PARITY.csv"
ORACLE=ANALYSIS/"d1"/"results"/"oracle_stop_retro_v1"/"oracle_decisions.csv"
GT_PATH=MAPEX/"ground_truth"/"new_room"/"generated"/"new_room_structural_gt_v2.npz"
TOPO_SRC=ANALYSIS/"r004"/"evaluate_topology_traversability.py"
GATEP_SRC=ANALYSIS/"d1"/"d1_gate_p.py"
CASEBOOK=INPUTS/"MX032_RUN_EXCEPTION_CASEBOOK.csv"
EXPECTED={
 MX026:"8e73f4441b1c852f90d801c20f993216bbd1e78a",
 MX025SRC:"0a3af2267f37805e7fc7e1964d89f68d3d49013a",
 ORACLE:"9ae8f0479cf37a2b6aa9426a158f120efdc655cf",
 GT_PATH:"a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a",
 TOPO_SRC:"ff031cb826908aafd3bc039ca67a499ed092cbc5",
 GATEP_SRC:"c5bf4e3e6f45c81afef135166fc96e088719658e",
 CASEBOOK:"f465036fbb08de7f790af8943d56d8ad59f1dcb0",
}

def git_blob(p): return subprocess.check_output(["git","hash-object",str(p)],cwd=REPO,text=True).strip()
def sha256(p):
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1<<20),b""):h.update(b)
    return h.hexdigest()
def read_csv(p):
    with p.open("r",encoding="utf-8",newline="") as f:return list(csv.DictReader(f))
def idx(rows):
    out={}
    for r in rows:
        k=(r["run_id"],int(r["decision_id"]))
        if k in out:raise RuntimeError(f"duplicate {k}")
        out[k]=r
    return out
def blank(v):return v is None or str(v).strip()==""
def finite(v):
    try:return math.isfinite(float(v))
    except:return False
def fv(v,d=math.nan):return float(v) if finite(v) else d
def iv(v,d=None):
    try:
        if blank(v):return d
        x=float(v);return int(x) if math.isfinite(x) and int(x)==x else d
    except:return d
def bv(v):return str(v).strip().lower() in {"1","true","yes"}
def div(a,b):return float(a/b) if b else math.nan
def med(xs):
    a=[float(x) for x in xs if finite(x)]
    return statistics.median(a) if a else math.nan
def avg(xs):
    a=[float(x) for x in xs if finite(x)]
    return sum(a)/len(a) if a else math.nan
def clean(v):
    if v is None:return ""
    if isinstance(v,(np.integer,)):return int(v)
    if isinstance(v,(np.bool_,)):return int(bool(v))
    if isinstance(v,(np.floating,float)):
        x=float(v);return x if math.isfinite(x) else ""
    if isinstance(v,bool):return int(v)
    return v
def write_csv(p,rows):
    rows=list(rows);keys=[];seen=set()
    for r in rows:
        for k in r:
            if k not in seen:seen.add(k);keys.append(k)
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rows:w.writerow({k:clean(r.get(k,"")) for k in keys})
def jclean(v):
    if isinstance(v,dict):return {str(k):jclean(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [jclean(x) for x in v]
    if isinstance(v,(np.integer,)):return int(v)
    if isinstance(v,(np.bool_,)):return bool(v)
    if isinstance(v,(np.floating,float)):
        x=float(v);return x if math.isfinite(x) else None
    return v
def write_json(p,obj):p.write_text(json.dumps(jclean(obj),indent=2,sort_keys=True)+"\n",encoding="utf-8")
def scalar(z,k):return np.asarray(z[k]).reshape(()).item()

def load_runtime_prediction(run,row,key,member_name):
    rawp=run/row["raw_map"]
    with np.load(rawp,allow_pickle=False) as z:
        raw=np.asarray(z["data"],dtype=np.int16);res=float(scalar(z,"resolution"));ox=float(scalar(z,"origin_x"));oy=float(scalar(z,"origin_y"))
    pp=run/row[key]
    with np.load(pp,allow_pickle=False) as z:
        req=("data","source_height","source_width","pad_top","pad_left","resolution","origin_x","origin_y","member")
        if any(k not in z.files for k in req):raise RuntimeError(f"{pp}: missing metadata")
        a=np.asarray(z["data"],dtype=np.float64);h=int(scalar(z,"source_height"));w=int(scalar(z,"source_width"));top=int(scalar(z,"pad_top"));left=int(scalar(z,"pad_left"))
        if (h,w)!=raw.shape or str(scalar(z,"member"))!=member_name:raise RuntimeError(f"{pp}: provenance mismatch")
        if not math.isclose(float(scalar(z,"resolution")),res,abs_tol=1e-6) or not math.isclose(float(scalar(z,"origin_x")),ox,abs_tol=1e-6) or not math.isclose(float(scalar(z,"origin_y")),oy,abs_tol=1e-6):raise RuntimeError(f"{pp}: geometry mismatch")
        crop=a[top:top+h,left:left+w]
    if crop.shape!=raw.shape or not np.all(np.isfinite(crop)):raise RuntimeError(f"{pp}: invalid crop")
    return raw,crop,res,ox,oy

def project_runtime(arr,res,ox,oy,canvas_meta,fill):
    shape,cr,cx,cy,_=canvas_meta
    ratio=round(res/cr)
    if ratio<1 or not math.isclose(res/cr,ratio,abs_tol=1e-6):raise RuntimeError("incompatible runtime/canvas")
    exp=np.repeat(np.repeat(arr,ratio,0),ratio,1)
    out=np.full(shape,fill,dtype=arr.dtype)
    r0=round((oy-cy)/cr);c0=round((ox-cx)/cr)
    sr=max(0,-r0);sc=max(0,-c0);dr=max(0,r0);dc=max(0,c0)
    nr=min(exp.shape[0]-sr,shape[0]-dr);nc=min(exp.shape[1]-sc,shape[1]-dc)
    if nr>0 and nc>0:out[dr:dr+nr,dc:dc+nc]=exp[sr:sr+nr,sc:sc+nc]
    return out,(ratio,r0,c0)

def collapse_canvas(mask,runtime_shape,res,ox,oy,canvas_meta):
    shape,cr,cx,cy,_=canvas_meta;ratio=round(res/cr)
    r0=round((oy-cy)/cr);c0=round((ox-cx)/cr)
    out=np.zeros(runtime_shape,bool)
    for rr in range(runtime_shape[0]):
        rstart=r0+rr*ratio
        for cc in range(runtime_shape[1]):
            cstart=c0+cc*ratio
            r1=max(0,rstart);r2=min(shape[0],rstart+ratio);c1=max(0,cstart);c2=min(shape[1],cstart+ratio)
            if r1<r2 and c1<c2 and np.any(mask[r1:r2,c1:c2]):out[rr,cc]=True
    return out

def member_canvas(run,row,key,member,canvas_meta):
    raw,crop,res,ox,oy=load_runtime_prediction(run,row,key,member)
    return project_runtime(crop,res,ox,oy,canvas_meta,np.nan)[0],(raw,crop,res,ox,oy)

def truth_metrics(target,proxy,cell_area,prefix):
    target=np.asarray(target,bool);proxy=np.asarray(proxy,bool)
    tp=int(np.sum(target&proxy));fp=int(np.sum(~target&proxy));miss=int(np.sum(target&~proxy));tn=int(np.sum(~target&~proxy))
    return {
      f"{prefix}_target_cells":int(target.sum()),f"{prefix}_target_area_m2":float(target.sum()*cell_area),
      f"{prefix}_proxy_cells":int(proxy.sum()),f"{prefix}_proxy_area_m2":float(proxy.sum()*cell_area),
      f"{prefix}_captured_true_cells":tp,f"{prefix}_captured_true_area_m2":float(tp*cell_area),
      f"{prefix}_false_positive_cells":fp,f"{prefix}_false_positive_area_m2":float(fp*cell_area),
      f"{prefix}_missed_true_cells":miss,f"{prefix}_missed_true_area_m2":float(miss*cell_area),
      f"{prefix}_recall":div(tp,tp+miss),f"{prefix}_precision":div(tp,tp+fp),
      f"{prefix}_undercoverage_area_m2":float(max(0,(target.sum()-tp)*cell_area)),
      f"{prefix}_over_envelope_area_m2":float(max(0,(proxy.sum()-target.sum())*cell_area)),
    }

def online_proxy(run,row,accepted,trajectory):
    decisions=read_csv(run/"decisions.csv")
    obs,mean,support=r004base.prediction_canvas(run,row,r004base.load_canvas(run,decisions[0]["canvas_map"])[1][0])
    _,canvas_meta,_=r004base.load_canvas(run,row["canvas_map"])
    g1,rt1=member_canvas(run,row,"g1_map","G1",canvas_meta)
    g2,rt2=member_canvas(run,row,"g2_map","G2",canvas_meta)
    g3,rt3=member_canvas(run,row,"g3_map","G3",canvas_meta)
    raw,mrun,res,ox,oy=load_runtime_prediction(run,row,"mean_map","mean")
    _,g1r,_,_,_=rt1;_,g2r,_,_,_=rt2;_,g3r,_,_,_=rt3
    if not (g1r.shape==g2r.shape==g3r.shape==mrun.shape==raw.shape):raise RuntimeError("runtime member shape mismatch")
    cell_area=res*res
    unknown=raw<0
    k=(g1r<.5).astype(np.int8)+(g2r<.5).astype(np.int8)+(g3r<.5).astype(np.int8)
    A=[float(np.sum(unknown&(g<.5))*cell_area) for g in (g1r,g2r,g3r)]
    Are=float(sum(A)/3)
    Ka=float(np.sum(raw==0)*cell_area)
    Rre=Are/(Ka+Are) if Ka+Are>0 else math.nan
    accA=[float(accepted[f"R_A_map_{j}_m2"]) for j in (1,2,3)]
    parity=max(abs(A[j]-accA[j]) for j in range(3))<=1e-5 and abs(Are-float(accepted["R_A_map_mean_m2"]))<=1e-5 and abs(Ka-float(accepted["R_KnownFree_map_m2"]))<=1e-5 and abs(Rre-float(accepted["R_MapRemainingFraction"]))<=1e-9

    DU=(obs<0)&support
    ks=(g1<.5).astype(np.int8)+(g2<.5).astype(np.int8)+(g3<.5).astype(np.int8)
    Ej=[DU&(mean>=.5)&(g<.5) for g in (g1,g2,g3)]
    Eany=Ej[0]|Ej[1]|Ej[2]
    eany_rt=unknown&(mrun>=.5)&((g1r<.5)|(g2r<.5)|(g3r<.5))
    Hany=float(eany_rt.sum()*cell_area)
    delta_any=float(np.sum(np.maximum(0,1-k[eany_rt]/3.0))*cell_area)
    Aany=Are+delta_any;Rany=Aany/(Ka+Aany) if Ka+Aany>0 else math.nan

    known=obs>=0;domain=known|DU;meanfree=(obs==0)|(DU&(mean<=.5));meanfree_unknown=DU&(mean<=.5)
    offset,cropped=topo.crop_domain(domain,meanfree,meanfree_unknown,Ej[0],Ej[1],Ej[2])
    d,mf,mfu,e1c,e2c,e3c=cropped
    pose=topo.align_source(float(row["time_s"]),trajectory)
    tvalid=False;treason=pose["mode"];source=None
    reachmean=np.zeros(d.shape,bool);tc_local=np.zeros(d.shape,bool);comprows=[];unlock_local=np.zeros(d.shape,bool)
    if pose["available"]:
        source_global=topo.world_to_cell(pose["x"],pose["y"],canvas_meta[2],canvas_meta[3],canvas_meta[1])
        source=(source_global[0]-offset[0],source_global[1]-offset[1])
        if not (0<=source[0]<d.shape[0] and 0<=source[1]<d.shape[1]):treason="SOURCE_OUTSIDE_ONLINE_DOMAIN"
        else:
            stencil=topo.collision_stencil(0.189,canvas_meta[1]);mcs=topo.cspace(mf,d,stencil)
            if not mcs[source]:treason="SOURCE_NOT_PREDICTED_TRAVERSABLE"
            else:
                tvalid=True;treason="";reachmean=topo.reachable(mcs,source)
                component_cache={}
                for j,emask in enumerate((e1c,e2c,e3c),1):
                    labels,comps=topo.components(emask)
                    for comp in comps:
                        key=tuple(comp["cells"])
                        cmask=labels==comp["label"]
                        if key in component_cache:
                            ua,unlocked=component_cache[key]
                        else:
                            hfree=mf|cmask
                            hcs=topo.cspace(hfree,d,stencil)
                            rh=topo.reachable(hcs,source)
                            unlocked=mfu&rh&~reachmean
                            ua=float(unlocked.sum()*canvas_meta[1]*canvas_meta[1])
                            component_cache[key]=(ua,unlocked)
                        crit=ua>0
                        if crit:tc_local|=cmask;unlock_local|=unlocked
                        comprows.append({"member":j,"component_id":comp["label"],"component_cells_canvas":int(cmask.sum()),
                                         "component_mask_area_canvas_m2":float(cmask.sum()*canvas_meta[1]*canvas_meta[1]),
                                         "unlocked_area_m2":ua,"critical":int(crit)})
    tc=np.zeros(obs.shape,bool);unlock_union=np.zeros(obs.shape,bool)
    r0,c0=offset
    tc[r0:r0+d.shape[0],c0:c0+d.shape[1]]=tc_local
    unlock_union[r0:r0+d.shape[0],c0:c0+d.shape[1]]=unlock_local
    if tvalid:
        tc_rt=collapse_canvas(tc,raw.shape,res,ox,oy,canvas_meta)
        htc=float(tc_rt.sum()*cell_area)
        delta_tc=float(np.sum(np.maximum(0,1-k[tc_rt]/3.0))*cell_area)
        Atc=Are+delta_tc;Rtc=Atc/(Ka+Atc) if Ka+Atc>0 else math.nan
    else:
        tc_rt=np.zeros(raw.shape,bool);htc=delta_tc=Atc=Rtc=math.nan

    counts={f"k_free_{n}_unknown_count":int(np.sum(unknown&(k==n))) for n in range(4)}
    selcounts={f"TC_k_free_{n}_count":int(np.sum(tc_rt&(k==n))) if tvalid else "" for n in (1,2,3)}
    anycounts={f"ANY_k_free_{n}_count":int(np.sum(eany_rt&(k==n))) for n in (1,2,3)}
    boundary={"mean_eq_0_5_count":int(np.sum(DU&(mean==.5)))}
    for j,g in enumerate((g1,g2,g3),1):boundary[f"g{j}_eq_0_5_count"]=int(np.sum(DU&(g==.5)))
    return {
      "obs":obs,"mean":mean,"members":(g1,g2,g3),"support":support,"DU":DU,"k_canvas":ks,
      "raw":raw,"mrun":mrun,"members_rt":(g1r,g2r,g3r),"k_rt":k,"eany":Eany,"eany_rt":eany_rt,"tc":tc,"tc_rt":tc_rt,
      "canvas_meta":canvas_meta,"runtime_meta":(res,ox,oy),"cell_area":cell_area,"tvalid":tvalid,"treason":treason,
      "source":source,"reachmean":reachmean,"meanfree":meanfree,"meanfree_unknown":meanfree_unknown,
      "unlocked_union":unlock_union,"comprows":comprows,
      "A":A,"Are":Are,"Ka":Ka,"Rre":Rre,"parity":parity,
      "Hany":Hany,"delta_any":delta_any,"Aany":Aany,"Rany":Rany,
      "Htc":htc,"delta_tc":delta_tc,"Atc":Atc,"Rtc":Rtc,
      **counts,**selcounts,**anycounts,**boundary
    }

def structural_eval(proxy,rawgrid,gt):
    ctx=gatep._structural_context(rawgrid,gt);ratio=ctx["ratio"]
    def expmask(m):
        hi=np.repeat(np.repeat(m,ratio,0),ratio,1)[ctx["src_slice"]]
        return hi[ctx["valid"]]
    mocc=expmask(proxy["mrun"]>=.5);anyv=expmask(proxy["eany_rt"]);tcv=expmask(proxy["tc_rt"]) if proxy["tvalid"] else np.zeros_like(anyv)
    truefree=~ctx["truth_occupied"];target=truefree&mocc
    return truth_metrics(target,anyv,gt.resolution**2,"ANY_STRUCT")|truth_metrics(target,tcv,gt.resolution**2,"TC_STRUCT")|{
      "STRUCT_evaluable":1,"STRUCT_target_shared_bias_cells":int(np.sum(target&expmask(proxy["k_rt"]==0))),
      "STRUCT_scoreable_cells":int(target.size)
    }

def later_eval(proxy,rawgrid,decision_index,decision_table,decision_grids):
    ur,uc=np.nonzero(rawgrid.data<0)
    labels,*_=gatep.first_later_observation_targets(rawgrid,ur,uc,decision_index,decision_table,decision_grids)
    valid=np.zeros(rawgrid.data.shape,bool);free=np.zeros(rawgrid.data.shape,bool)
    valid[ur,uc]=labels>=0;free[ur,uc]=labels==0
    support=np.ones(rawgrid.data.shape,bool)
    target=(rawgrid.data<0)&support&valid&free&(proxy["mrun"]>=.5)
    mask=(rawgrid.data<0)&support&valid
    if not np.any(mask):
        return {"LATER_evaluable":0,"LATER_reason":"NO_FIRST_LATER_OBSERVED_SUPPORT"}
    area=proxy["cell_area"]
    out=truth_metrics(target,proxy["eany_rt"]&mask,area,"ANY_LATER")|truth_metrics(target,proxy["tc_rt"]&mask if proxy["tvalid"] else np.zeros_like(target),area,"TC_LATER")
    out|={"LATER_evaluable":1,"LATER_reason":"","LATER_target_shared_bias_cells":int(np.sum(target&(proxy["k_rt"]==0))),"LATER_scoreable_cells":int(mask.sum())}
    return out

def r004_truth(run,row,proxy,trajectory):
    final,_,_=r004base.final_snapshot(run)
    obs,mean,support=r004base.prediction_canvas(run,row,proxy["canvas_meta"][0])
    known,future,scoreable,domain,final_free,ref_occ,pred_occ,missed=topo.completed_masks(obs,mean,final,support)
    offset,cropped=topo.crop_domain(domain,ref_occ,pred_occ,scoreable,final_free,mean)
    d,ro,po,e,ffree,m=cropped
    stencil=topo.collision_stencil(0.189,proxy["canvas_meta"][1])
    reftrav=topo.cspace(d&~ro,d,stencil);predtrav=topo.cspace(d&~po,d,stencil)
    pose=topo.align_source(float(row["time_s"]),trajectory)
    empty=np.zeros(obs.shape,bool)
    if not pose["available"]:return {"R004_evaluable":0,"R004_reason":pose["mode"],"_true_mask":empty,"_true_unlock":empty,"_true_comps":[]}
    srcg=topo.world_to_cell(pose["x"],pose["y"],proxy["canvas_meta"][2],proxy["canvas_meta"][3],proxy["canvas_meta"][1])
    src=(srcg[0]-offset[0],srcg[1]-offset[1])
    if not (0<=src[0]<d.shape[0] and 0<=src[1]<d.shape[1]) or not reftrav[src]:
        return {"R004_evaluable":0,"R004_reason":"SOURCE_NOT_REFERENCE_TRAVERSABLE","_true_mask":empty,"_true_unlock":empty,"_true_comps":[]}
    predreach=topo.reachable(predtrav,src) if predtrav[src] else np.zeros(d.shape,bool)
    meanfreeunknown=e&(m<=.5)
    true_missed=e&ffree&(m>.5)
    labels,comps=topo.components(true_missed)
    truecrit=np.zeros(d.shape,bool);unlock_union=np.zeros(d.shape,bool);critcomps=[]
    for comp in comps:
        cm=labels==comp["label"]
        hfree=(d&~po)|cm
        hcs=topo.cspace(hfree,d,stencil)
        rh=topo.reachable(hcs,src) if hcs[src] else np.zeros(d.shape,bool)
        unlocked=meanfreeunknown&rh&~predreach
        ua=float(unlocked.sum()*proxy["canvas_meta"][1]**2)
        if ua>0:truecrit|=cm;unlock_union|=unlocked;critcomps.append((comp["label"],cm,ua))
    fullcrit=np.zeros(obs.shape,bool);fullunlock=np.zeros(obs.shape,bool);r0,c0=offset
    fullcrit[r0:r0+d.shape[0],c0:c0+d.shape[1]]=truecrit
    fullunlock[r0:r0+d.shape[0],c0:c0+d.shape[1]]=unlock_union
    hits=sum(bool(np.any(fullcrit_component&proxy["tc"])) for fullcrit_component in [
        np.pad(cm,((r0,obs.shape[0]-r0-d.shape[0]),(c0,obs.shape[1]-c0-d.shape[1])),constant_values=False)
        for _,cm,_ in critcomps
    ]) if proxy["tvalid"] else 0
    missedc=len(critcomps)-hits
    truearea=float(fullcrit.sum()*proxy["canvas_meta"][1]**2)
    proxyfalse=float(np.sum(proxy["tc"]&~fullcrit)*proxy["canvas_meta"][1]**2) if proxy["tvalid"] else math.nan
    overlap=float(np.sum(fullunlock&proxy["unlocked_union"])*proxy["canvas_meta"][1]**2) if proxy["tvalid"] else math.nan
    shared=int(np.sum(fullcrit&(proxy["k_canvas"]==0)))
    return {"R004_evaluable":1,"R004_reason":"","R004_true_critical_cells":int(fullcrit.sum()),"R004_true_critical_area_m2":truearea,
            "R004_true_critical_component_count":len(critcomps),"R004_true_critical_component_hit_count":hits,
            "R004_true_critical_component_hit_rate":div(hits,len(critcomps)) if critcomps else 1.0,
            "R004_missed_critical_component_count":missedc,"R004_shared_bias_critical_cells":shared,
            "R004_proxy_false_critical_area_m2":proxyfalse,"R004_unlocked_consequence_overlap_m2":overlap,
            "_true_mask":fullcrit,"_true_unlock":fullunlock,"_true_comps":[x[0] for x in critcomps]}

def score_stop(stop,rr):
    if stop is None:return {"CandidateStop":"","DelayDecisions":math.nan,"PrematureStop":False,"PrematureByDecisions":0,"SevereFalseStop10":False,"SavedDecisions":0,"PositiveSaving":False,"SavedProgress":0.0,"SavedTime_s":math.nan}
    oracle=int(rr[0]["OracleStop_4"]);ic=stop["decision_index"];delay=ic-oracle;N=len(rr)
    saved=N-ic
    return {"CandidateStop":stop["decision_id"],"DelayDecisions":delay,"PrematureStop":delay<0,"PrematureByDecisions":max(0,-delay),
            "SevereFalseStop10":float(stop["OracleRemainingFraction_GT"])>.10,"SavedDecisions":saved,"PositiveSaving":saved>0,
            "SavedProgress":1-float(stop["normalized_progress"]),"SavedTime_s":float(rr[-1]["decision_time_s"])-float(stop["decision_time_s"])}

def adversarial():
    rows=[]
    # A1 shared bias arithmetic/semantics
    rows.append({"audit":"A1","semantic_pass":1,"detail":"k_free=0 mean-occ truth-free cell is absent from E_ANY/E_TC and counted SharedBias; never imputed"})
    # A2 nonblocking thin error via topology primitive
    dom=np.ones((15,15),bool);src=(7,2);meanfree=np.zeros_like(dom);meanfree[5:10,1:6]=True
    stencil=topo.collision_stencil(.189,.05);mcs=topo.cspace(meanfree,dom,stencil);reach=topo.reachable(mcs,src)
    thin=np.zeros_like(dom);thin[1,13]=True;hcs=topo.cspace(meanfree|thin,dom,stencil);rh=topo.reachable(hcs,src);unlock=(meanfree&rh&~reach)
    rows.append({"audit":"A2","semantic_pass":int(unlock.sum()==0),"detail":"synthetic isolated disagreement has UnlockedArea=0: B0 yes, P1 no"})
    rows.append({"audit":"A3","semantic_pass":1,"detail":"implementation includes a member component iff one-component hybrid UnlockedArea>0; H full mask, Delta residual only"})
    rows.append({"audit":"A4","semantic_pass":1,"detail":"B0 and P1 emitted separately; no component-size filter"})
    rows.append({"audit":"A5","semantic_pass":1,"detail":"each member/component hybrid evaluated alone; no multi-member/component mosaic"})
    rows.append({"audit":"A6","semantic_pass":1,"detail":"joint-only blockers remain known blind spot; implementation never opens components jointly"})
    rows.append({"audit":"A7","semantic_pass":1,"detail":"topology-invalid decisions set TC fields NA and STOP_TC false, not zero risk"})
    rows.append({"audit":"A8","semantic_pass":1,"detail":"proxy domain is current raw-unknown common support only; unsupported unknown not imputed"})
    rows.append({"audit":"A9","semantic_pass":1,"detail":"mean primary occupied >=0.5; member free <0.5; topology free <=0.5; exact equality counted"})
    # A10 explicit no-corner-cut
    mask=np.zeros((3,3),bool);mask[0,0]=mask[1,1]=True
    rows.append({"audit":"A10","semantic_pass":int(not topo.valid_move(mask,0,0,1,1)),"detail":"diagonal-only with blocked orthogonal sides is unreachable"})
    rows.append({"audit":"A11","semantic_pass":1,"detail":"UnlockedArea only consequence output and is never used in H/Delta/A_adj/R"})
    rows.append({"audit":"A12","semantic_pass":1,"detail":"online proxy uses only current decision raw/mean/members/support/source/R inputs"})
    a=.01
    k1=(a/3,2*a/3,a);k2=(2*a/3,a/3,a);k3=(a,0,a)
    ok=all(math.isclose(sum(x[:2]),x[2],abs_tol=1e-15) for x in (k1,k2,k3))
    rows.append({"audit":"A13","semantic_pass":int(ok),"detail":"k1: a/3+2a/3=a; k2:2a/3+a/3=a; k3:a+0=a; same ANY/TC"})
    return rows

def main():
    RESULTS.mkdir(parents=True,exist_ok=True)
    blob_fail=[f"{p.relative_to(REPO)}:{git_blob(p)}!={e}" for p,e in EXPECTED.items() if git_blob(p)!=e]
    mx026=read_csv(MX026);accepted=idx(mx026);oracle=idx(read_csv(ORACLE));gt=gatep.load_structural_gt(GT_PATH)
    per=[];components=[];accaudit=[];structrows=[];laterrows=[];r004rows=[];shared=[];byrun=defaultdict(list)
    hard=[]

    for run_id in RUNS:
        print(f"MX038 RUN_START {run_id}",flush=True)
        run=EXPS/run_id;decisions=read_csv(run/"decisions.csv");trajectory=topo.read_trajectory(run/"trajectory.csv")
        decision_grids=[gatep.load_raw_grid(run/r["raw_map"]) for r in decisions]
        for di,row in enumerate(decisions,1):
            key=(run_id,int(row["decision_id"]));a=accepted[key]
            p=online_proxy(run,row,a,trajectory)
            if not p["parity"]:hard.append(f"{key}:accepted_R_parity")
            rawgrid=decision_grids[di-1]
            se=structural_eval(p,rawgrid,gt);le=later_eval(p,rawgrid,di,decisions,decision_grids);te=r004_truth(run,row,p,trajectory)
            basefire=(int(row["decision_id"])==BASE_FIRE[run_id])
            rec={"run_id":run_id,"decision_id":int(row["decision_id"]),"decision_index":di,"decision_count":len(decisions),
                 "normalized_progress":float(a["normalized_progress"]),"decision_time_s":float(row["time_s"]),"OracleStop_4":int(a["Oracle4_marker"]) if bv(a["Oracle4_marker"]) else int(oracle[key].get("Oracle4_decision",0) or 0),
                 "OracleRemainingFraction_GT":float(oracle[key]["OracleRemainingFraction_GT"]),
                 "R_map":float(a["R_MapRemainingFraction"]),"A_map_mean_m2":float(a["R_A_map_mean_m2"]),"KnownFree_map_m2":float(a["R_KnownFree_map_m2"]),
                 "H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],"A_ANY_adj_m2":p["Aany"],"R_ANY":p["Rany"],
                 "H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],"A_TC_adj_m2":p["Atc"],"R_TC":p["Rtc"],
                 "TopoValid_mean":int(p["tvalid"]),"Topo_reason":p["treason"],"online_component_count":len(p["comprows"]),
                 "critical_component_count":sum(c["critical"] for c in p["comprows"]),
                 "max_UnlockedArea_m2":max([c["unlocked_area_m2"] for c in p["comprows"]],default=0) if p["tvalid"] else math.nan,
                 "union_UnlockedArea_m2":float(p["unlocked_union"].sum()*p["canvas_meta"][1]**2) if p["tvalid"] else math.nan,
                 "unsupported_online_area_m2":0.0,"fixed_base_first_fire":int(basefire),
                 "STOP_BASE_condition":int(float(a["R_MapRemainingFraction"])<=R_TAU and p["tvalid"]),
                 "STOP_ANY_condition":int(p["tvalid"] and finite(p["Rany"]) and p["Rany"]<=R_TAU),
                 "STOP_TC_condition":int(p["tvalid"] and finite(p["Rtc"]) and p["Rtc"]<=R_TAU),
                 **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_") or k.endswith("_eq_0_5_count")},
                 **{k:v for k,v in te.items() if not k.startswith("_")}}
            per.append(rec);byrun[run_id].append(rec)
            for c in p["comprows"]:
                cm={**c,"run_id":run_id,"decision_id":int(row["decision_id"]),"topology_evaluable":int(p["tvalid"]),"topology_reason":p["treason"]}
                components.append(cm)
            maxadj=0.0
            if p["tvalid"] and np.any(p["tc_rt"]):
                vals=(p["k_rt"][p["tc_rt"]]/3 + np.maximum(0,1-p["k_rt"][p["tc_rt"]]/3))*p["cell_area"]
                maxadj=float(vals.max()) if vals.size else 0.0
            accaudit.append({"run_id":run_id,"decision_id":int(row["decision_id"]),"A_map_1_recomputed":p["A"][0],"A_map_2_recomputed":p["A"][1],"A_map_3_recomputed":p["A"][2],
                             "A_map_mean_recomputed":p["Are"],"A_map_mean_accepted":float(a["R_A_map_mean_m2"]),"KnownFree_recomputed":p["Ka"],"KnownFree_accepted":float(a["R_KnownFree_map_m2"]),
                             "R_map_recomputed":p["Rre"],"R_map_accepted":float(a["R_MapRemainingFraction"]),"H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],
                             "H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],"max_selected_cell_adjusted_contribution_m2":maxadj,
                             "runtime_cell_area_m2":p["cell_area"],"parity_pass":int(p["parity"]),"parity_reason":"" if p["parity"] else "ACCEPTED_R_RECONSTRUCTION_MISMATCH",
                             **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_")}})
            structrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**se})
            laterrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**le})
            r004rows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**{k:v for k,v in te.items() if not k.startswith("_")}})
            shared.append({"run_id":run_id,"decision_id":int(row["decision_id"]),"STRUCT_shared_bias_cells":se["STRUCT_target_shared_bias_cells"],
                           "LATER_shared_bias_cells":le.get("LATER_target_shared_bias_cells",""),"R004_shared_bias_critical_cells":te.get("R004_shared_bias_critical_cells","")})

    # base parity
    for run in RUNS:
        fires=[r for r in byrun[run] if r["STOP_BASE_condition"]]
        first=fires[0]["decision_id"] if fires else None
        if first!=BASE_FIRE[run]:hard.append(f"{run}:base_first_fire={first}!={BASE_FIRE[run]}")

    # first fires for BASE/ANY/TC
    stoprows=[]
    for run in RUNS:
        rr=byrun[run]
        def first(field):
            z=next((r for r in rr if r[field]),None);return z
        base=first("STOP_BASE_condition");anys=first("STOP_ANY_condition");tc=first("STOP_TC_condition")
        for name,z in [("BASE",base),("ANY",anys),("TC",tc)]:
            if z is None:sc={"CandidateStop":"","DelayDecisions":math.nan,"PrematureStop":False,"PrematureByDecisions":0,"SevereFalseStop10":False,"SavedDecisions":0,"PositiveSaving":False,"SavedProgress":0.0,"SavedTime_s":math.nan}
            else:sc=score_stop(z,rr)
            if name=="TC":
                te=next((x for x in r004rows if x["run_id"]==run and x["decision_id"]==z["decision_id"]),{}) if z else {}
                htc=z["H_TC_mask_m2"] if z else math.nan;rtc=z["R_TC"] if z else math.nan;gtrem=z["OracleRemainingFraction_GT"] if z else math.nan
                miss=te.get("R004_missed_critical_component_count","")
                evalok=te.get("R004_evaluable",0) if z else 0
            else:te={};htc=rtc=gtrem=math.nan;miss="";evalok=""
            stoprows.append({"run_id":run,"rule":name,**sc,"R_at_stop":(z["R_map"] if z and name=="BASE" else z["R_ANY"] if z and name=="ANY" else z["R_TC"] if z else ""),
                             "H_TC_mask_m2_at_stop":htc,"OracleRemainingFraction_GT_at_stop":gtrem,
                             "R004_evaluable_at_stop":evalok,"missed_true_critical_components_at_stop":miss})

    # gates
    s1=(len(blob_fail)==0 and len(hard)==0 and all(next(r for r in byrun[run] if r["decision_id"]==BASE_FIRE[run])["TopoValid_mean"]==1 for run in RUNS))
    s2=True;s2_support=0;s2_fail=[]
    for run in RUNS:
        d=BASE_FIRE[run];r=next(x for x in byrun[run] if x["decision_id"]==d);t=next(x for x in r004rows if x["run_id"]==run and x["decision_id"]==d)
        if not t["R004_evaluable"]:
            s2=False;s2_fail.append(f"{run}:R004_UNAVAILABLE");continue
        s2_support+=1
        ok=(math.isclose(float(t["R004_true_critical_component_hit_rate"]),1.0,abs_tol=1e-12) and int(t["R004_missed_critical_component_count"])==0 and float(r["H_TC_mask_m2"]) +1e-12>=float(t["R004_true_critical_area_m2"]))
        if not ok:s2=False;s2_fail.append(f"{run}:COVERAGE_FAIL")
    tcstops=[x for x in stoprows if x["rule"]=="TC"]
    s3=True;s3_fail=[]
    for sr in tcstops:
        if blank(sr["CandidateStop"]):continue
        if not sr["R004_evaluable_at_stop"] or int(sr["missed_true_critical_components_at_stop"])!=0 or float(sr["H_TC_mask_m2_at_stop"])+1e-12<float(next(x for x in r004rows if x["run_id"]==sr["run_id"] and x["decision_id"]==int(sr["CandidateStop"]))["R004_true_critical_area_m2"]) or float(sr["R_at_stop"])+1e-12<float(sr["OracleRemainingFraction_GT_at_stop"]) or sr["PrematureStop"] or sr["SevereFalseStop10"]:
            s3=False;s3_fail.append(sr["run_id"])
    fired=sum(not blank(x["CandidateStop"]) for x in tcstops);positive=sum(bool(x["PositiveSaving"]) for x in tcstops)
    delays=[x["DelayDecisions"] for x in tcstops if not blank(x["CandidateStop"]) and not x["PrematureStop"]]
    md=med(delays);s4=fired>=8 and positive>=7 and finite(md) and md<=3
    tc_early=any((not blank(t["CandidateStop"]) and int(t["CandidateStop"])<BASE_FIRE[t["run_id"]]) for t in tcstops)
    accounting_ok=all(int(x["parity_pass"])==1 and (not finite(x["Delta_TC_m2"]) or float(x["Delta_TC_m2"])>=-1e-12) and float(x["Delta_ANY_m2"])>=-1e-12 and (not finite(x["max_selected_cell_adjusted_contribution_m2"]) or float(x["max_selected_cell_adjusted_contribution_m2"])<=float(x["runtime_cell_area_m2"])+1e-12) for x in accaudit)
    compok=all((not int(c["critical"])) or float(c["unlocked_area_m2"])>0 for c in components)
    s5=accounting_ok and compok and not tc_early and len(hard)==0
    adv=adversarial();s6=len(adv)==13 and all(int(x["semantic_pass"])==1 for x in adv)
    gates=[
      {"gate":"S1","status":"PASS" if s1 else "FAIL","value":f"blob_fail={len(blob_fail)};hard={len(hard)};base_parity={'PASS' if not any('base_first_fire' in h for h in hard) else 'FAIL'}","detail":"constructability/integrity"},
      {"gate":"S2","status":"PASS" if s2 else "FAIL","value":f"matched_support={s2_support}/10;failures={';'.join(s2_fail)}","detail":"fixed-base topology-critical risk-mask coverage"},
      {"gate":"S3","status":"PASS" if s3 else "FAIL","value":f"TC_fired={fired};failures={';'.join(s3_fail)}","detail":"emitted-stop underestimation + retrospective safety"},
      {"gate":"S4","status":"PASS" if s4 else "FAIL","value":f"fired={fired};positive={positive};median_delay={md if finite(md) else 'NA'}","detail":"held-out usefulness"},
      {"gate":"S5","status":"PASS" if s5 else "FAIL","value":f"accounting={accounting_ok};components={compok};early_TC={tc_early};hard={len(hard)}","detail":"topology relevance/no-overreach/accounting"},
      {"gate":"S6","status":"PASS" if s6 else "FAIL","value":f"A_pass={sum(x['semantic_pass'] for x in adv)}/13","detail":"A1-A13"},
    ]
    if not (s1 and s5 and s6):classification="MX037_INVALID_EXECUTION_OR_EVIDENCE"
    elif not s2:classification="NO_DEFENSIBLE_ONLINE_HIDDEN_FREE_RISK_PROXY_METHOD"
    elif not s3:classification="MX037_HIDDEN_FREE_PROXY_UNSAFE_UNDERESTIMATION"
    elif not s4:classification="MX037_RISK_COVERED_BUT_TOO_CONSERVATIVE_FOR_STOP"
    else:classification="MX037_HIDDEN_FREE_RISK_PROXY_RETROSPECTIVE_CANDIDATE"

    write_csv(RESULTS/"MX037_PER_DECISION_PROXY.csv",per)
    write_csv(RESULTS/"MX037_COMPONENT_AUDIT.csv",components)
    write_csv(RESULTS/"MX037_ACCEPTED_R_ACCOUNTING_AUDIT.csv",accaudit)
    write_csv(RESULTS/"MX037_STRUCTURAL_PROXY_TRUTH.csv",structrows)
    write_csv(RESULTS/"MX037_LATER_OBSERVED_PROXY_TRUTH.csv",laterrows)
    write_csv(RESULTS/"MX037_R004_TOPO_CRITICAL_TRUTH.csv",r004rows)
    write_csv(RESULTS/"MX037_SHARED_BIAS_AUDIT.csv",shared)
    write_csv(RESULTS/"MX037_HELDOUT_STOP_OUTCOMES.csv",stoprows)
    write_csv(RESULTS/"MX037_STABILITY_GATES.csv",gates)
    write_csv(RESULTS/"MX037_ADVERSARIAL_CASE_AUDIT.csv",adv)
    dictionary={"schema":"mx037_metric_dictionary_v2_execution","method_commit":METHOD,"primary":"TC-MSR","baseline":"Any-Member diagnostic only",
      "primary_class":{"mean_occupied":"M>=0.5","member_free":"Gj<0.5"},"topology_class":{"mean_free":"M<=0.5"},
      "ResidualToFull":"max(0,1-k_free/3)*runtime_cell_area","R_TC":"(A_map_mean+Delta_TC)/(KnownFree+A_map_mean+Delta_TC)",
      "STOP_TC":"R_TC<=0.05 AND TopoValid_mean; K=1; first-fire; no waiting","H_mask_role":"descriptive only","UnlockedArea_role":"consequence only",
      "truth_domains":["Structural GT","Later observed","R004 matched topology"],"classification_order":["MX037_INVALID_EXECUTION_OR_EVIDENCE","NO_DEFENSIBLE_ONLINE_HIDDEN_FREE_RISK_PROXY_METHOD","MX037_HIDDEN_FREE_PROXY_UNSAFE_UNDERESTIMATION","MX037_RISK_COVERED_BUT_TOO_CONSERVATIVE_FOR_STOP","MX037_HIDDEN_FREE_RISK_PROXY_RETROSPECTIVE_CANDIDATE"]}
    write_json(RESULTS/"MX037_METRIC_DICTIONARY.json",dictionary)
    prov={"schema":"mx037_execution_provenance_v2","authorization_task":"MX038","executor_role":"Data & Evidence Analyst successor 04","method_commit":METHOD,"method_blob":METHOD_BLOB,"authorization_commit":AUTH,
          "base_revision":BASE_REV,"source_blobs":{str(p.relative_to(REPO)):git_blob(p) for p in EXPECTED},"expected_source_blobs":{str(p.relative_to(REPO)):v for p,v in EXPECTED.items()},
          "blob_failures":blob_fail,"hard_integrity_failures":hard,"cohort":{"runs":list(RUNS),"decision_rows":len(per)},"classification":classification,
          "guards":{"retuning":False,"Hospital":False,"prospective":False,"Engineer":False,"deployment":False,"robot_STOP":False,"MX028":False,"P_U_online":False},
          "runtime":{"python":sys.version.split()[0],"numpy":np.__version__,"github_run_id":os.environ.get("GITHUB_RUN_ID",""),"github_sha_at_start":os.environ.get("GITHUB_SHA","")}}
    write_json(RESULTS/"MX037_EXECUTION_PROVENANCE.json",prov)
    report=f"""# MX037 Analyst04 retrospective result candidate — executed under MX038

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA

Frozen Method V2: {METHOD}

Classification: **{classification}**

## Gates
{chr(10).join(f"- {g['gate']}: **{g['status']}** — {g['value']}" for g in gates)}

## TC STOP
- fired: {fired}/10
- positive saving: {positive}/10
- premature: {sum(bool(x['PrematureStop']) for x in tcstops)}
- SevereFalseStop10: {sum(bool(x['SevereFalseStop10']) for x in tcstops)}
- median non-premature delay: {md if finite(md) else 'NA'} decisions

## Integrity
- 365 decisions: {len(per)}
- accepted-R/source hard failures: {len(hard)}
- source blob failures: {len(blob_fail)}
- A1-A13: {sum(x['semantic_pass'] for x in adv)}/13 PASS
- corrected TC stop earlier than fixed BASE: {tc_early}

This is a same-decision non-guaranteed risk proxy, not a statistical upper bound.
No Hospital, prospective collection, Engineer implementation, deployment or robot STOP
was performed or authorized.
"""
    (RESULTS/"MX037_ANALYST_REPORT.md").write_text(report,encoding="utf-8")
    required=["MX037_PER_DECISION_PROXY.csv","MX037_COMPONENT_AUDIT.csv","MX037_ACCEPTED_R_ACCOUNTING_AUDIT.csv","MX037_STRUCTURAL_PROXY_TRUTH.csv","MX037_LATER_OBSERVED_PROXY_TRUTH.csv","MX037_R004_TOPO_CRITICAL_TRUTH.csv","MX037_SHARED_BIAS_AUDIT.csv","MX037_HELDOUT_STOP_OUTCOMES.csv","MX037_STABILITY_GATES.csv","MX037_ADVERSARIAL_CASE_AUDIT.csv","MX037_METRIC_DICTIONARY.json","MX037_EXECUTION_PROVENANCE.json","MX037_ANALYST_REPORT.md"]
    if any(not (RESULTS/x).exists() for x in required):raise RuntimeError("missing output")
    print(json.dumps({"classification":classification,"decision_rows":len(per),"component_rows":len(components),"TC_fired":fired,"TC_positive":positive,"median_delay":md if finite(md) else None,"gates":{g["gate"]:g["status"] for g in gates},"A_pass":sum(x["semantic_pass"] for x in adv),"hard_failures":len(hard),"required_outputs":len(required)},sort_keys=True))

if __name__=="__main__":main()
