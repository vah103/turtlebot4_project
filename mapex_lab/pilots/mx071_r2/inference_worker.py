"""Per-member inference worker: no GT, no resize, no online planner."""
import argparse
import json
from pathlib import Path
import resource

from native_adapter import predict_member,torch
import numpy as np

p=argparse.ArgumentParser()
p.add_argument("member",type=int)
p.add_argument("input")
p.add_argument("output")
p.add_argument("timing")
a=p.parse_args()
torch.manual_seed(0)
observed=np.load(a.input)
prediction,timing=predict_member(observed,a.member)
np.save(a.output,prediction)
timing["maxrss_mib"]=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.
Path(a.timing).write_text(json.dumps(timing))
