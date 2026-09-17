# mpx_w2_001

Purpose: MapEx Way2 integration/sanity run.

This run did terminate by the frozen Way2 early-stopping rule at policy decision 29.

Known recorder issue:
`summary.json` records `termination_reason = exploration_complete`, but the
decision logs correctly record `way2_early_stop`. This recorder-only issue was
fixed after this run and did not affect Way2 policy behavior or stopping time.

This run is an implementation sanity run, not prospective validation data.
