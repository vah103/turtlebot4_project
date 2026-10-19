# MapEx probabilistic raycast accumulator: source-faithful audit

Date: 2026-10-19 | Environment: New Room P1, MapEx/LaMa 3 model CPU

## Source-level defect and controlled correction

Original source: /home/dell/MapEx/scripts/simple_mask_utils.py,
the Numba implementation get_free_points(). It initializes accum_hit_prob=0
inside the for-cell loop, so a ray stops only on an individual cell with
p(occupied)>=0.8, not when accumulated risk along the ray passes 0.8.

Paper-faithful experiment moves the same initialization one nesting level
up to occur exactly once per ray. All else remains unchanged: 250 rays,
20m predicted visibility, shapely polygon buffer, same MapEx flood_fill_simple,
LaMa three-checkpoint mean/variance, 0.1m P1, original WFD, Euclidean score,
native A*, G1 V4-R4.1, SWEEP-R1 frozen local variant.

No original source file is edited. The original Numba source is SHA-256
74f13cc8f013b26c733067fc99811cc001d37ac356bece86e53fcccf4a5ec576;
the runtime patch checks this identity before applying the one-line relocation.

## Offline 25-decision validation

Source is the sealed MapEx-only COM1 Run05:
  /home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob

Replay uses ORIGINAL simple_mask_utils.get_vis_mask and flood_fill_simple
and the same unknown intersection / ensemble variance summed in native
MapEx cost scorer. Unlike the previous approximate replay, it does not
substitute an approximate polygon or alternate flood fill.

Original-code replay selects the original recorded winner for ALL 25/25
decisions. Recalculated cost arrays have some nonzero discrepancy at
early decisions (worst max absolute score residual 0.074188); this is
because the sealed evidence contains only the P1 core 203x263, not all
LaMa padding-border prediction pixels. Later decisions from D12 onward
match original costs to within 1e-6. Consequently, this is strict
winner-choice validation, NOT full bit-exact scorer-value reproduction.

Fixed-input cumulative-ray replay changes winner for exactly 11/25
historical observation states:
D02, D05, D06, D09, D10, D11, D14, D17, D18, D19, D21.
Equivalent proportion: 44%. This is an *offline controlled scoring
counterfactual*, NOT the number of changed decisions for a newly executed
robot trajectory, since observations will diverge after the first change.

## Native closed-loop experiment (immutable staged sources)

New Run IDs:
- NR-RAYMX-02: Original MapEx only + corrected accumulator
- NR-RAYRN-02: Original MapEx + G1 V4-R4.1 + corrected accumulator
- NR-RAYSW-02: Original MapEx + G1 + exact frozen G2-SWEEP-R1
  LOCAL + corrected accumulator

Every branch keeps the same New Room original P1 map, initial robot pose,
three-model optimized LaMa, raycast range 20m, original A*.
No Hospital or Gazebo inference is claimed. Exact final coverage is
measured from native final saved known-cell masks. Run status must be
SOURCE_TERMINAL with returncode 0 to be accepted. If an unfinished or
guard-aborted run occurs, report it openly rather than compare as complete.

The D01/D02 guard for this new policy checks new-run data integrity,
NOT numerical parity with old raycast behavior. Changing the raycast
deliberately invalidates old-trajectory parity; manifest records this.

The same delta is introduced across all three variants. No resource
overlap: all three runs execute sequentially on DELL.
