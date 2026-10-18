"""Coverage-and-distance certificate before overriding original G2-R6.2 leg.

This deliberately rejects count-only frontier swaps; supports R3 D03/D05
regression evidence. Proposed goal MUST be on native-certified observed FREE.
An isolated first-leg certificate is necessary but not sufficient to commit a
replacement for a multi-goal route. Never chooses a coordinate using future.
"""
from __future__ import annotations

def pathwise_dominance(original_path, alternative_path, orig_nom, orig_stress,
                       alt_nom, alt_stress):
    if not original_path or not alternative_path:
        return {'allow':False,'reason':'EMPTY_OR_UNREACHABLE_PATH'}
    if len(alternative_path)>len(original_path):
        return {'allow':False,'reason':'ALTERNATIVE_LONGER'}
    lost_nom=int((orig_nom & ~alt_nom).sum())
    lost_stress=int((orig_stress & ~alt_stress).sum())
    if lost_nom or lost_stress:
        return {'allow':False,'reason':'LOST_PREDICTED_ROOM_VISIBILITY',
                'lost_nominal_cells':lost_nom,'lost_stress_cells':lost_stress}
    return {'allow':True,'reason':'PATHWISE_DOMINATES_ORIGINAL',
            'saved_route_cells':len(original_path)-len(alternative_path),
            'lost_nominal_cells':0,'lost_stress_cells':0}
