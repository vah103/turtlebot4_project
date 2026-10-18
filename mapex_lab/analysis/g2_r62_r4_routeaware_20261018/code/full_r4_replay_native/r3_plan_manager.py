"""R6.2 R3 light replanning and observation-task tracking."""
import time
import numpy as np
from r61_plan_manager import R61PlanManager,RouteProposal
from r61_structural_events import digest
from r3_frontier import choose

class R3PlanManager(R61PlanManager):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.r3_target_witness={}
        self.r3_last_choice=None

    def register_choice(self,decision):
        self.r3_last_choice=decision
        if decision.source=="FRONTIER_RESIDUAL" and decision.goal is not None:
            self.r3_target_witness[decision.goal]=tuple(decision.witness_rc)

    def propose_light(self):
        if self.pending_full or self.monitor.pending_level in (
                "S0_SAFETY_HOLD","S3_STRUCTURAL_REBUILD","S4_EXIT_LOCAL"):
            return {"action":"WAIT_STRUCTURAL_REFRESH"}
        began=time.perf_counter()
        obs=self.monitor.observed
        pose=self.monitor.pose
        scope=np.asarray(self.last_native_mask,bool)&(obs==0)&self.r6.room
        if not scope[pose]:
            self.pending_full=True
            return {"action":"WAIT_NATIVE_SAFETY"}
        try:
            c=choose(obs,self.r6.mean,self.r6.var,self.r6.semantic,pose,scope)
        except (ValueError,RuntimeError) as exc:
            return {"action":"DEFER_LIGHT_ROUTE","reason":str(exc)[:180]}
        seq=self.monitor.obs_seq
        self.cached_proposal=RouteProposal(
            obs_seq=seq,event_no=self.monitor.event_no,room_id=self.room_id,
            pose=pose,observed_hash=digest(obs),goals=c.suggested_goal_chain,
            distance_m=float(c.first_route_m),
            solve_seconds=time.perf_counter()-began)
        self.register_choice(c)
        return {"action":"LIGHT_PROPOSAL",
                "source_obs_seq":seq,
                "goal_sequence_rc":[list(g) for g in c.suggested_goal_chain],
                "distance_m":round(c.first_route_m,3),
                "compute_s":round(self.cached_proposal.solve_seconds,4),
                "R3_source":c.source,
                "R3_frontier_covered":c.frontier_covered,
                "R3_frontier_total":c.frontier_total,
                "RoomNode_rerun":False,"LaMa_rerun":False,
                "still_needs_native_ASTAR":True}
