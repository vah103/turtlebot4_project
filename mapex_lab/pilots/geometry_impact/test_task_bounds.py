"""Exhaustive small-tree tests for resource-stop and relaxed-prefix bounds."""
import itertools
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
sys.path.insert(0,'/home/dell/mapex_verification2d_deps')
from task_search import BoundFrontier,meets,TaskEvaluator
import numpy as np

checks=0
target=(80,9,10,90)
assert meets(target,target)
assert not meets((85,8,10,92),target)
assert not meets((85,9,10,89),target)
assert not meets((79,10,10,100),target)
assert meets((80,0,0,90),target)
assert not meets(target,(80,0,0,90))
checks+=6

# Actual attainable actions are {A,B}; relaxed action C can appear to achieve
# the target earlier but must never inflate the certified lower bound.
tree={0:(0,False,True,[1,2,3]),
      1:(20,False,True,[4,5]),2:(30,False,True,[6]),3:(10,False,False,[7]),
      4:(70,True,True,[]),5:(90,True,True,[]),6:(60,True,True,[]),7:(25,True,False,[])}
truth=100-min(d for d,hit,attainable,_ in tree.values() if hit and attainable)
for order in itertools.permutations([1,2,3]):
    for cap in range(9):
        tracker=BoundFrontier(100)
        tracker.add(0,0)
        todo=[0]
        for step in range(cap):
            if not todo:
                break
            node=todo.pop(0)
            d,hit,attainable,children=tree[node]
            if hit:
                tracker.hit(node,d,attainable)
            else:
                children=list(order) if node==0 else children
                for child in children:
                    tracker.add(child,tree[child][0])
                    # An interruption before parent close must remain safe.
                    low,upper=tracker.interval()
                    assert low<=truth<=upper
                    checks+=1
                tracker.close(node)
                todo.extend(children)
            low,upper=tracker.interval()
            assert low<=truth<=upper
            checks+=1
        low,upper=tracker.interval()
        assert low<=truth<=upper
        if not todo:
            assert low==truth and upper==75
        checks+=1

root=Path(__file__).resolve().parent
dependencies=root.parent/'visibility_impact_r4/dependencies'
if not dependencies.exists():
    dependencies=root.parent/'visibility_r4_src/dependencies'
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module
core=load(dependencies/'active_verification_2d/core.py','test_task_core')
fast=load(dependencies/'fast_bfs.py','test_task_fast')
occupied=np.zeros((20,20),dtype=bool)
occupied[0,:]=occupied[-1,:]=occupied[:,0]=occupied[:,-1]=True
occupied[1:15,10]=True
observed=np.full(occupied.shape,.5,dtype=np.float32)
observed[0,:]=observed[-1,:]=observed[:,0]=observed[:,-1]=1
observed[6,5]=0
engine=SimpleNamespace(core=core,fast=fast,cfg={'robot_radius_m':.15,'resolution_m':.1})
asset={'start':np.array([6,5]),'occupied':occupied,'domain':np.ones_like(occupied)}
evaluator=TaskEvaluator(engine,asset,2026100809)
evaluator.goals=[(6,15)]*100
world=SimpleNamespace(observed=observed)
mean=np.zeros(occupied.shape,dtype=np.float32)
mean[observed!=.5]=observed[observed!=.5]
bad=evaluator.counts(world,mean)
good=evaluator.counts(world,occupied.astype(np.float32))
assert bad[3]==0   # A route exists in the prediction but crosses a GT wall.
assert good[3]==100 # A valid footprint-safe route goes around that wall.
assert good[1]==good[2] and good[0]==bad[0]
checks+=3
print({'status':'PASS_TASK_FRONTIER_BOUNDS_AND_QUALITY','checks':checks,'exact_true_gain':truth,'relaxed_upper_gain':75,'route_test':[bad[3],good[3]]})
