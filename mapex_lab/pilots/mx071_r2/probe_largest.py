"""Full largest-map memory gate; same source, model, transform and CPU contract."""
import json
from pathlib import Path
import resource
from native_adapter import ROOT,atomic_json,make_mapper,predict_member,sha_array
import numpy as np

ART=ROOT/"artifacts"
manifest=json.loads((ART/"manifest.json").read_text())
selected=max(manifest["maps"],key=lambda m:np.prod(m["shape"]))
with np.load(str(ART/(selected["map_id"]+"_sealed.npz"))) as z:
    gt=z["gt"]
mapper=make_mapper(gt)
pose=selected["starts"][0]["padded_row_col"]
mapper.observe_and_accumulate_given_pose(pose)
observed=mapper.obs_map.astype(np.float32)
print("LARGEST_MAP_MEMBER_START",selected["map_id"],list(observed.shape),flush=True)
prediction,timing=predict_member(observed,1)
np.savez_compressed(str(ART/"largest_probe_prediction.npz"),prediction=prediction,
                    observed_hash=sha_array(observed))
atomic_json(ART/"largest_probe.json",dict(status="PASS",map_id=selected["map_id"],
                                         timing=timing,
                                         maxrss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.))
print("LARGEST_MAP_MEMBER_OK",json.dumps(timing),flush=True)
