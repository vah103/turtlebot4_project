# Resources, lossless state and preflight — MX071 follow-up R2

Part of PLAN_R2.md. These are **new proposed R2 bounds**, not inherited MX071/MX072 acceptance. All probes/tests below await R2 methodology ACCEPT and explicit technical authorization. No probe was run to write or correct this file.

Bounded correction after review0038e276: #3 exposure matching is charged inside the existing diagnostic/RSS/archive limits; #8 is claim-only. No requests, model-forward groups, quota, forecast clock or technical fixture allowances are increased. Focused closure remains pending; resource/runtime remains uncertified.

## 1. Read-only DELL facts and feasibility status

Observed during drafting, via metadata/git-object reads only:
- Device DELL b5efb201-1f65-4be7-b57f-1dc0b00aaf35.
- Intel i3-7100U, four logical CPUs; MemTotal 7,994,752KiB (about7.62GiB).
- About6.61GiB filesystem space available to the account; contemporaneous MemAvailable about4.59GiB, fluctuating with other work.
- Four checkpoint files together 3,606,919,939 bytes (about3.36GiB). Hashes are the prior audited identities, subject to runtime recheck.
- Full-size old **PIPE_ALIGNED** evidence includes G1 VmHWM4725.40MiB and first-map G1–G3 sum158.25s. It does not measure the native four-model R2 pipeline, lossless decoder, new weighted-controller paths or full study duration.

**CURRENT FULL/STAGED EXECUTION FEASIBILITY: NOT CERTIFIED.** Disk currently fails both proposed option gates. Memory availability is not presently enough to guarantee the proposed isolated peak. Do not delete old evidence, stop unrelated applications, move to COM1 or invent a new archive service to make this look ready.

## 2. Hard scientific and technical ceilings

| Limit | FULL_7 | STAGED_CORE alternative |
| --- | --- | --- |
| Native acquisitions | 8 | 8 |
| All-seven Stage A slots | 32 S +32 O | same |
| Scientific Stage B requests before dedup | 176 | 64 (#1/#4/#8 only) |
| Full KEEP suffix seal requests | 16 | 16 |
| Total replay requests | 192 | 80 |
| Source + replay traces, maximum | 200 | 88 |
| Horizon repetitions/action | 1; G10/Q10 reuse full trace | 1 |
| Automatic infrastructure retries | 0 | 0 |
| Native control iterations/trace | original remainder of1000 | same |
| Technical fresh-prediction batches/trace | <=64 | <=64 |
| Stage A O shadow batches | <=64 (32 O x2) | same |
| Future technical preflight groups | <=128 | <=128 |
| Source/model decoder work groups, maximum | 12,992 | 5,824 |
| Model forward invocations, including one decoder verification | <=103,936 | <=46,592 |
| Study storage quota | 48GiB | 32GiB |
| Required extra free space, reserved before start | quota +2GiB =50GiB | 34GiB |
| Cumulative elapsed-work stop loss | 100 days | 45 days |
| Model worker concurrency | 1 | 1 |
| Worker kernel peak RSS | <=5120MiB | same |
| Coordinator peak RSS | <=512MiB | same |
| Minimum available RAM during work | >=640MiB | same |
| Cold-start admission available RAM | >=6272MiB (worker+coordinator+margin) | same |
| Source geometry/model tensor area | <=4,194,304 cells; no resize | same |
| Native candidates/epoch | <=256; no truncation | same |
| #6 expanded pool | <=1024 | not executed |
| Nominated cache blocks | <=96 x128MiB | same |

A work group is one four-model batch, its native numerical aggregation, required scoring/cache encoding, and **one independent lossless decoder check**, thus up to eight model forward invocations. The preflight allowance uses this same maximum. Native scalar/scan/planning-only work is separately bounded below.

Arithmetic:
- FULL: 200x64 +64 +128 =12,992 groups; x8=103,936 forwards.
- STAGED: 88x64 +64 +128 =5,824 groups; x8=46,592 forwards.
These intentionally conservative bounds assume no branch reuse and no early terminal.
- Scientific per-question request caps: 16+16+40+16+24+32+32=176.
- STAGED_CORE: 16+16+32=64.
The old75/80 ceiling is not reused.

The **64-batch guard is a technical resource limit**, not a changed native mission/termination rule. If a valid native continuation needs batch65 before its scientific cutoff, mark RESOURCE_INFERENCE_CAP / TECHNICAL_ABORT and censor Q. Do not carry it forward as a native terminal or call the missing questions answered.

The protocol is bounded; it does not guarantee every branch finishes. Admission must disclose projected support risk and actual upper bound before execution. The 100/45-day maxima are emergency conservative limits, not requests to silently allocate months of DELL time.

## 3. Cost calculation and two fully costed options

Before science, measure genuine cold/full-size groups on **each of the four source shapes**, all G/G1/G2/G3, native preprocessing/scoring and exact decoder. Let g_plus be max witnessed verified-group wall x1.5, rounded up to whole seconds. Require g_plus<=600s, every member forward/load<=1200s, and no identity/parity/RSS failure.

A finite fixture maximum is not a proof of all future latency. Treat it as the admission forecast, impose runtime per-group600s plus cumulative stop loss, and report any exceeded bound as technical censoring.

Also require:
- Non-model control/scan/planning/recipe verification bound c_plus<=2.00s/control record, from full-size source fixtures. This is an admission estimate with runtime deadline, not a theoretical promise.
- Extra nominated-diagnostic work outside the groups d_plus<=600s/unique slot, max64 slots. Time all #1/#3/#4/#6 renderer and proposal work, including expanded pools and the full #3 baseline exposure trace, exact-key/dose/M_j matching and audit. #3 permits at most one additional baseline render/candidate, <=256/S, <=8192 across32 S slots; these are diagnostic renderer calls, not new model groups or scientific branches. Reuse exact baseline intermediates when available. If the existing600s/RSS bound fails, return technical/support censoring rather than expanding the budget or dropping exposure fields; no stand-in resource substitution.
- Analysis/final lossless audit reservation48h full,24h staged. Model decoders counted in groups cannot be hidden in this reservation.
- One group in flight and at most two bounded bundles for encoded/decoded byte comparison. No model+decoder workers overlapping to evade RSS.
- Actual elapsed/resume totals recorded even for failed work; no reset of the quota after process restart.

Forecast:
T_full =12,992*g_plus +200*1001*c_plus +64*d_plus +48h.
T_staged =5,824*g_plus +88*1001*c_plus +64*d_plus +24h.
CPU upper =3*T (two Torch threads plus one coordinator) with measured process CPU also reported.

Illustrative forecast scenarios, **not measurements**:

| Verified-group upper g_plus | Full seven-question bound, at c=2s,d=600s | Staged #1/#4/#8 bound |
| --- | --- | --- |
| 120s | about25.12 days | about11.57 days |
| 300s | about52.19 days | about23.71 days |
| 600s | about97.30 days | about43.93 days |

Admission additionally requires actual forecast<=100d full or45d staged, requested policy costs/precision support and no per-trace technical guard already predicted to censor the nominated tests. Report best observed and conservative bound separately; do not tell USER this illustrative table is an ETA.

**Option FULL_7:** shared all-seven Stage A and all seven paired contracts,176 scientific requests;48GiB quota; maximum costs above. Requires at least50GiB newly reserved free space on an authorized DELL storage location and genuine preflight. It answers all seven only where valid support meets gates.

**Option STAGED_CORE:** same eight native sources and all-seven diagnostic screen, then paired #1/#4/#8 only,64 scientific requests;32GiB quota and34GiB free. #2/#3/#5/#6 lack their causal task-loss/cost answers. It cannot claim all-seven complete. Any later extension is a fresh bounded execution decision, not an automatic use of saved compute.

The current requested design is FULL_7. If resource/preflight cannot support it, PM presents both options and the excluded answers to USER. This drafting action selects neither a scientific execution budget nor a narrower accepted scope.

## 4. Lossless archive contract

No quantization, tensor resize/crop, sparse candidate omission, snapshot thinning or failed-run deletion. A compressed/reconstructed object is acceptable only if its decoder reproduces **the exact original bytes and required alias/state semantics**.

Use a study-local content-addressed store (CAS), typed object manifest and append-only event/request ledger. Keys bind byte hash, dtype, shape, layout, source/frame/phase and dependencies. Map/config/weights are shared immutable roots, not copied per branch. CAS aliases save storage, not statistical N.

### 4.1 What is stored verbatim or losslessly encoded

- Raw asset/config/source/model identities and immutable model roots; allowance4GiB for the four checkpoint files. Used configs/shared libraries/env/archive format are hash-bound.
- All nominated S/O caches and the immediate-pre O caches: <=96 blocks, each<=128MiB, including policy-relevant G0/G2 auxiliary/native binary extraction, three ensemble channel-0 maps, mean/V, exact input/mask, live pool/score/cache/control state and complete decoder roots.
- Canonical source categorical arrays can use a2-bit alphabet with exact dtype/value reconstruction; float predictions remain exact float bits. Noncanonical values require raw/lossless float representation. No unreported rounding or conversion.
- Model input channel/alias repetition can share exact identical byte blocks. Source dictionary aliasing must be preserved; a copied semantic map is insufficient.
- Scalars/arrays/candidates/paths/scans include sizes, dtype, shape, SHA256 and complete lossless representation or verified reconstruction recipe; original retry/deletion/hit ordering and duplicates are retained.
- All invalid/zero/NA/censored requests and logs. No successful-only manifest.

### 4.2 Exact reconstruction recipes

For non-nominated future states, retain model input roots, actual physical world, each independently executed pose/control sequence, source state graph, RNG/model state and all output hashes. Recipes can reconstruct:
- PyOMap and physical scan outputs from exact GT/origin/renderer/libraries.
- Ordered accum_hit_points from ordered per-scan output chunks; duplicates are not collapsed.
- Native A* path from the exact observed planning map/options/library and selected target.
- Tensor outputs from pinned frozen models and byte-identical input/RNG/context.
- Candidate flood masks/masses from their exact mean/V/U/renderer/origin and operation ordering.

**For every emitted recipe object, encoder performs one fresh decode and compares every required array byte and state-graph reference before committing the manifest.** Model decode cost is included in the eight-forward work group. Physical/path/render decode is metered research work. A recipe is not an assertion that a vaguely similar re-run should work.

Persist complete observed input roots/ordered event ancestry; an alternate's scan recipe uses its **own** executed pose and map history. Baseline future sensor outputs are not branch input.

On any non-identical decoder output: store/retain the original object if space permits, mark LOSSLESS_CODEC_FAIL, stop the affected study and return for bounded correction. Do not use raw fallback to silently exceed quota or certify missing outcomes. Raw full-fleet fallback would require a separately costed/reviewed storage amendment.

Byte-bound buckets for FULL_7:
- Nominated caches12GiB.
- Shared weights/config/source/GT/env roots6GiB.
- Control/event ledger <=200x1001x16KiB <3.06GiB.
- Candidate epoch blocks <=200x64x256x1KiB =3.125GiB, plus <=128MiB all extra nominated proposals/controls. Repeated cached pools are references, not discarded candidates.
- Additional state graphs, diagnostics, failures, timers and analysis <=4GiB, **including** #3 baseline exposure records <=2GiB. At most32 S states x64MiB; ROI<=64x441=28,224 cells, nativeF<=256. The per-candidate (free/first-blocker/hit count, visible/origin bits) vector uses three uint16 counts plus flags/reserved byte=8bytes:28,224x256x8=57,802,752bytes before bounded ROI/key/match headers, below64MiB. Keep variance/input refs rather than duplicate full arrays. Fixed binary vector keys with byte comparison after hash lookup prevent an unbounded text/object expansion. Stream one native candidate mask/ray list and obey the existing worker/decoder/coordinator RSS caps. The whole4GiB bucket remains binding; exceeding it is technical abort, not increased quota or omitted records.
- Working byte-comparison/spool/staging <=4GiB.
- Audit output, checksum/manifest reserve<=4GiB.
- Remaining quota margin >11GiB.
All buckets are runtime-counted; total includes temporary duplication and partial writes. STAGED has the same cache/root requirements but88 traces and smaller ledger/work/audit allowances, verified total<=32GiB. A exceeded per-object bound blocks commitment; no field dropping.

Crash-safe manifests reference only complete flushed/hash-verified objects. Partial temp files remain accounted and marked. Old MX071 raw data/models are protected, not moved/deleted to fit this design. No unconfigured remote archive is assumed.

## 5. Model residency and state fidelity

One isolated CPU model at a time; Torch threads2, interop1; no gradient/training. Preserve source input/mask and correct G/member forward order.

Sequential reload is an explicit implementation adaptation:
- Capture canonical logical model initialization, complete loaded parameter/buffer hash and any missing/unexpected checkpoint keys. strict=False is not permission to leave live random parameters unbound.
- Preserve the source RNG progression around incidental reload; do not consume new RNG state in the source stream just because the implementation reloads a model.
- Capture immediate G auxiliary values before later forwards mutate a shared batch dict. Respect array/dictionary alias graphs.
- Compare direct member forward and the source-call interpreter, same full input and model state. Compare stack/mean/variance/native scalar reduction bytes and forward-state hashes.
- No output/channel clipping except native auxiliary visualization conversion. Frozen/eval does not by itself prove deterministic byte replay.
- Hash actual environment: Python3.6.13/Torch1.10.2 CPU expected from prior DELL context, but verify executable/version and native-library binaries. A version mismatch is not assumed safe.

Kernel VmHWM/RUSAGE peak must be measured in fresh real runner and decoder processes, plus coordinator overlap and system available memory. Sampled RSS alone is insufficient. Unknown-blocked/cropped ALIGNED probes and MX0724.5GiB acceptance do not transfer.

Expanded-pool scoring may stream exact per-candidate masks instead of retaining the whole source visualization mask list. Preserve candidate/argmin/retry order, scalar reduction bytes, all archived masks/recipes and the native best/third-mask outputs where consumed. This explicit memory adaptation needs source-call parity and liveness proof; it cannot skip a candidate, coarsen a mask or change source arithmetic. Incidental unused mapper arrays may be constant recipes only after the same liveness proof. Scoring runs in the bounded worker, not an unmetered coordinator.

## 6. Required independent technical preflight contracts

Each result is PASS/FAIL/NA with exact runner/source/weights/env/fixture IDs. Stand-in results cannot close real-model checks.

| ID | Required check and failure action |
| --- | --- |
| P01 | All raw source/config/model/map/start identities; native config DT=True, mission1000 and effective unknownFalse; fail blocks all |
| P02 | Full-size tensor shape/frame, pad offsets/unshifted origins, default_map_eval/no resize; P1/P2 valid-space/coverage component hashes; fail blocks affected cohort |
| P03 | Real G/G1/G2/G3 direct-call versus sequential source-call outputs, alias order, loaded model/buffer/RNG state and sample variance, repeat bytes on every shape; fail blocks all |
| P04 | Reference native source-call control prefix versus instrumented headless runner with recorder off/on; candidate/order/pose/scan/cache/terminal equality; actual models, not stand-ins |
| P05 | Complete serializer liveness inventory and alias-graph/ordered hit reconstruction; every controlling source read covered; no unexplained field |
| P06 | Lossless model/input/map/mask/path/render object encoding and independent byte decode on full-size real states, including repeated cache inputs and mutations; fail stops archive/replay |
| P07 | Fresh runner plus decoder kernel RSS<=5120MiB, coordinator<=512MiB, availability/headroom; no process accumulation excuse without exact fresh-process evidence |
| P08 | Actual verified-group/control/diagnostic time and per-member load limit; all option forecasts include decoder, #6 expansion, #3 exposure tap/matching/audit and I/O within existing600s slot allowance; any bound failure blocks admission |
| P09 | Collision before append/scan, attempted endpoint trace, no-frontier/exhausted pool, invalid/nonfinite/zero path, source1000-step terminal; source behavior retained |
| P10 | Exact-at100 scan, before100 state, crossing endpoint and overshoot, duplicate-distance/zero-move scan, same Q/10m hold in both arms; no invented100m state |
| P11 | Locked/reached/invalid/unreachable goal, source retry ordering; post-scan O capture before next control; no eligibility-induced source mutation |
| P12 | #1 all-identical mask native mass parity; fractional masks/zero variance; #2 common U/clamping before-after and older-native-cache distinction |
| P13 | #3 port/P2/raster/dose/no-control/seed rules plus immutable pre-treatment exact native-pool exposure: tapped/untapped renderer outputs equal, ordered free/first-blocker/hit counts and full-mask/origin flags, exact full-key histogram matching, per-candidate M_j zero/5% bounds and one-pass neutral/random construction. Exercise an equally distant but occluded control, candidate-exposure mismatch, no-key/no-V/no-render support, insufficient match and no-redraw/no-alternate-patch dispositions; first-blocker/behind-ray/early-return/padding quirks preserved. #4 GT changes unknown-valid mean only, preserves U/V/known and native renderer gap labels |
| P14 | #5 KEEP==SHAM behavioral bytes; same-winner/full-cache identity, safe no-valid retarget KEEP; one intervention and native commitment release |
| P15 | #6 proposal order/near rejection/duplicate/no-refill/known-route and fixed-cluster versus expanded-pool contract; #8 exact action alias, unique/tied/descriptive alignment and tested-regret/disagreement labels. Even A0=unique G10 winner must never yield causal native short-horizon mechanism or online-recovery attribution |
| P16 | Fixed registry/192 ceiling, no alias N inflation, no split short/full rolls, all invalid/NA/technical cases, total cost after abort/resume |
| P17 | Censoring interval propagation, native absorbing versus technical missing, correct map/start macro and threshold/negative outcomes; no zero imputation |
| P18 | Total bytes including raw nominated data/working copies/failures/model roots; every object decoded and all candidate/failed records retained; future freeze/authority manifest |

Technical fixtures should exercise PRE_SELECT and active-lock POST_SCAN source phases, boundaries, failure/reselection states. They are non-scientific synthetic/control fixtures, declared as such and separately authorized. At most128 preflight work groups; exceeding this needs a bounded method/resource amendment, not an unlabeled batch.

After separately authorized Stage A acquisition, P19 is the **full suffix KEEP seal of every nominated Stage B state**, up to16, using native real models/world and exact event trace. This is additional to preflight prefixes and is included in the192 replay ceiling. One mismatch stops all alternate requests; P19 is not pre-certified by P03/P04.

## 7. Go/stop and implementation handback

Before acquisition: exact method ACCEPT, explicit engineering/preflight and scientific acquisition authority, all applicable P01–P18 PASS, option selected by PM/USER, actual reserved disk/RAM/time admission, immutable manifest and complete request template.

Before Stage B: P19 PASS for each state, full generated registry published before outcome execution, correct support/failure/alias accounting and explicit B authority. If only some states pass, do not silently operate the rest as a complete cohort; report partial/inconclusive support and return to the prescribed gate.

Immediate stop loss: wrong source/model/world identity; recorder/source mutation; replay divergence; non-lossless decoder; quota/RSS/available-memory/elapsed/group limit breach; corrupted manifest. Preserve prefixes and causal-missingness bounds. Do not replace maps, relax resource limits, tune thresholds, retry automatically or promote PIPE/Stage C.

Engineering's later deliverable is the exact native runner + serializer/decoder + tests/timers + admission forecast. Return it to independent technical review before collection. This design contains no runnable inference/collection entry point and is not an implementation ACCEPT.

