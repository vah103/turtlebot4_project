"""R4 temporary native inflated-start blocker, causal scan-triggered G1 recheck.

Never declares room coverage complete. Never issues navigation command and never
relaxes native inflation/A*. Only asks native MapEx to recompute G1 after the
robot has moved along its currently validated GLOBAL navigation route to a
safe planner start pose. No future observations, GT or arbitrary UNKNOWN count
thresholds.
"""
import numpy as np

class ExitGuardState(object):
    def __init__(self,pending=False,from_rc=None,room_mask=None,
                 started_native_tick=-1,source_reason='',rechecks=0):
        self.pending=bool(pending)
        self.from_rc=from_rc
        self.room_mask=room_mask
        self.started_native_tick=started_native_tick
        self.source_reason=source_reason
        self.rechecks=rechecks

def classify_exit(mode, local_wfd_centers, robot_rc, native_planner_grid,
                  accepted_room, current_tick):
    p=tuple(map(int,robot_rc))
    if mode != 'GLOBAL' or accepted_room is None:
        return ExitGuardState(), {'guard':'NOT_APPLICABLE'}
    if native_planner_grid is None or not accepted_room[p]:
        return ExitGuardState(), {'guard':'NO_CONFIRMED_ROOM_OR_PLANNER'}
    if len(local_wfd_centers)==0:
        return ExitGuardState(), {'guard':'OFFICIAL_WFD_EMPTY'}
    if np.isfinite(native_planner_grid[p]):
        return ExitGuardState(), {'guard':'VALID_START_WFD_UNREACHABLE_OTHER_CAUSE'}
    return ExitGuardState(True,p,accepted_room.copy(),int(current_tick),
                          'NATIVE_INFLATED_START_BLOCKED',0), {
        'guard':'TEMPORARY_START_CLEARANCE_BLOCKED',
        'official_wfd_candidate_count':len(local_wfd_centers),
        'robot_at_observed_free_but_native_inflation_invalid':True,
        'authorizes_local_exit':False,
        'reenter_only_via_fresh_G1_WFD':True,
    }

def maybe_recheck(state, robot_rc, inflated_grid, observed_free, min_motion_cells):
    """Return trigger only after certified safe native start and enough motion.

    Does not decide official room completion. Existing GLOBAL goal/path remains
    active until native MapEx replan executes.
    """
    if not state.pending:
        return False,'NO_PENDING_EXIT_RECHECK'
    p=tuple(map(int,robot_rc))
    room=state.room_mask
    if room is None or not (0<=p[0]<room.shape[0] and 0<=p[1]<room.shape[1]):
        state.pending=False
        return False,'ROOM_SCOPE_INVALID'
    if not room[p]:
        state.pending=False
        return False,'ROBOT_LEFT_ORIGINAL_ROOM'
    if not observed_free:
        return False,'ROBOT_NOT_OBSERVED_FREE'
    if not np.isfinite(inflated_grid[p]):
        return False,'NATIVE_START_STILL_INFLATED_BLOCKED'
    motion=float(np.linalg.norm(np.asarray(p,float)-np.asarray(state.from_rc,float)))
    if motion < min_motion_cells:
        return False,'WAIT_FOR_CAUSAL_MOVEMENT'
    state.pending=False
    state.rechecks+=1
    return True,'RECHECK_OFFICIAL_LOCAL_WFD_NOW'
