"""Finite reference-population estimators for R4; no cell-as-run inference."""
from __future__ import annotations
import hashlib
import numpy as np
from scipy.stats import hypergeom


def sample_flips(actions, cap, seed):
    """Census action changes first; sample without viewing rollout outcomes."""
    ids = sorted(i for i, values in actions.items() if len(set(values)) > 1)
    if not ids:
        return dict(T=len(actions), M=0, k=0, selected=[], inclusion_probability=0.)
    k = min(int(cap), len(ids))
    if k < 1:
        raise ValueError('Positive sample cap required')
    rng = np.random.default_rng(seed)
    selected = sorted(int(x) for x in rng.choice(ids, size=k, replace=False))
    return dict(T=len(actions), M=len(ids), k=k, selected=selected,
                inclusion_probability=k/len(ids), flip_ids=ids)


def population_rate_interval(M, k, successes, alpha=.05):
    """Invert finite-population hypergeometric tails, including zero hits."""
    if not (0 <= successes <= k <= M):
        raise ValueError('Invalid sample counts')
    if M == 0:
        return [0., 0.]
    if k == M:
        return [successes/M, successes/M]
    if k == 0:
        return [0., 1.]
    population_successes = np.arange(M+1)
    lo_ok = hypergeom.sf(successes-1, M, population_successes, k) >= alpha/2
    hi_ok = hypergeom.cdf(successes, M, population_successes, k) >= alpha/2
    possible = np.flatnonzero(lo_ok & hi_ok)
    return [float(possible[0]/M), float(possible[-1]/M)]


def estimate(T, M, effects, quality_diffs, delta=.02, quality_margin=.005,
             alpha=.05):
    """Zero-action-change states remain in T, not just sampled cases."""
    k = len(effects)
    if not (0 <= M <= T):
        raise ValueError('Invalid census counts')
    if T == 0:
        return dict(status='NO_VALID_DECISIONS', rate=None, mean_effect=None)
    if M == 0:
        return dict(status='CENSUS_ZERO_ACTION_CHANGE', rate=0.,
                    mean_effect=0., rate_interval=[0., 0.], mean_interval=[0., 0.])
    if not (0 < k <= M) or len(quality_diffs) != k:
        raise ValueError('Incomplete rollout sample; do not silently drop failures')
    x = np.asarray(effects, dtype=float)
    q = np.asarray(quality_diffs, dtype=float)
    if not np.isfinite(x).all() or not np.isfinite(q).all() or np.max(np.abs(x)) > 1+1e-9:
        raise ValueError('Invalid effect')
    h = (x >= delta) & (q >= -quality_margin)
    scaling = M/T
    rate_bounds = population_rate_interval(M, k, int(h.sum()), alpha)
    # Conservative bounded-mean interval; no unjustified narrow normal CI.
    width = 0. if k == M else np.sqrt(2*np.log(2/alpha)/k)
    return dict(status='CENSUS' if k == M else 'WEIGHTED_SUBSAMPLE',
                T=T, M=M, k=k, successes=int(h.sum()),
                rate=float(scaling*h.mean()), mean_effect=float(scaling*x.mean()),
                sampled_quality_differences=q.tolist(),
                sampled_effects=x.tolist(),
                rate_interval=[scaling*v for v in rate_bounds],
                mean_interval=[scaling*max(-1., float(x.mean()-width)),
                               scaling*min(1., float(x.mean()+width))],
                interval_scope='finite observed reference episode; not new buildings')


def stable_seed(seed, name):
    return int.from_bytes(hashlib.sha256((str(seed)+':'+name).encode()).digest()[:8], 'little')
