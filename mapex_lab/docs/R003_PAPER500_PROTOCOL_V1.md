# R003 — Approved New Room paper500 protocol

Date: 2026-09-23

Profile: `new_room_mapex_paper500_v1`; budget:
`odom_progress_0p30m_500step_v1`.

This protocol supersedes the paper1000 horizon for all new official R003
results. The frozen scientific semantics in `R003_PAPER1000_PROTOCOL_V1.md`
remain unchanged except for the explicit amendments below.

## Frozen paper500 horizon

- One adapted step is 0.30 m accumulated odometry translation.
- The maximum is 500 adapted steps, or 150.0 m.
- Common samples are initial, each crossing `k=10,20,...,500`, and the
  authoritative terminal sample.
- Natural completion may hold its final metric to 500. Algorithmic failure is
  never held.
- Primary reporting remains curve-first Coverage/IoU/TU with AUC;
  `Coverage@500`, `IoU@500`, and `TU@500` are secondary endpoints.
- The official namespace is `nf_p500_001..010` and
  `mpx_p500_001..010`. No p1000 attempt is part of this cohort.

## Runtime and retry gate

Official runtime is blocked until independent H030 Stage-B review ACCEPTS the
exact implementation SHA. A technically invalid physical-deadlock attempt may
be removed and repeated under the same run ID only when the frozen watchdog
fires. Poor metrics, normal completion, and ordinary algorithmic/navigation
outcomes are never retry reasons.

The watchdog is symmetric for NF and MapEx:

- action state must remain active;
- startup grace is 120 wall seconds;
- meaningful cumulative odometry progress is 0.05 m;
- no such progress for 180 continuous wall seconds declares technical
  deadlock;
- progress or inactive action state resets the window;
- missing odometry cannot declare deadlock.

The batch driver persists state atomically after every transition, resumes at
the first unfinished ID, allows at most one automatic retry per ID, and stops
on repeated or ambiguous technical invalidity.

