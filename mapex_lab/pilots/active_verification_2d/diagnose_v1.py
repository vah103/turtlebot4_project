"""Post-hoc V1 hypothesis-error audit; truth is evaluation-only."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np

from .core import PolicyInput
from .policy import hypotheses
from .run import write_json, write_csv

BASE = Path(__file__).resolve().parent


def main():
    cfg = json.loads((BASE/"protocol.json").read_text())
    output = BASE/"results/v2_diagnosis"; output.mkdir(exist_ok=True)
    rows = []
    for path in sorted((BASE/"results/pilot_v1/raw").glob("*/initial.npz")):
        case = path.parent.name; layout = case.split("__")[0]
        with np.load(path) as z:
            state = PolicyInput(z["observed"].copy(), z["mean"].copy(), z["variance"].copy(),
                                z["predictions"].copy(), tuple(z["pose"]), .1, 8.)
        with np.load(BASE/"assets"/(layout+".npz")) as z:
            truth = z["occupied"].copy()
        hs, _ = hypotheses(state, cfg)
        for h in hs:
            rr, cc = h.patch_cells.T
            changed = (state.mean[rr, cc] >= .5) == (h.intervention == "open")
            n = int(changed.sum())
            wrong = int(((state.mean[rr, cc] >= .5) != truth[rr, cc])[changed].sum())
            rows.append(dict(case=case, centre_row=h.centre[0], centre_col=h.centre[1],
                             intervention=h.intervention, impact_m2=h.impact_m2, changed_cells=n,
                             wrong_changed_cells=wrong, error_fraction=wrong/max(1,n), agreement=h.ensemble_agreement))
    write_csv(output/"hypothesis_errors.csv", rows)
    write_json(output/"scope.json", dict(status="POST_HOC_DEVELOPMENT_ONLY", source="pilot_v1", uses_truth=True,
                                        warning="Diagnostic labels must not enter online policy or confirmation selection."))
    print("AUDIT", len(rows), "initial hypotheses")


if __name__ == "__main__":
    main()
