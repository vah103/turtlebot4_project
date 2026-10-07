"""Small prediction-error estimator. Runtime features contain no world truth.

The logistic model is fitted OFFLINE on V1 development snapshots. Its output
is an error-risk estimate, not a certified/calibrated probability of a whole
geometric hypothesis. Cross-layout diagnostics accompany the fitted model.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi
from scipy.special import expit
from scipy.optimize import minimize

from .core import UNKNOWN

FEATURES = ["predicted_occupied", "soft_ambiguity", "ensemble_variance",
            "vote_disagreement", "distance_from_known", "local_edge_density"]


def feature_grid(state):
    mean = np.clip(state.mean, 0, 1)
    occupied = mean >= .5
    votes = np.mean(state.predictions >= .5, axis=0)
    local = ndi.uniform_filter(occupied.astype(float), size=5, mode="nearest")
    distance = np.minimum(ndi.distance_transform_edt(state.observed == UNKNOWN)
                          * state.resolution, 20.0) / 20.0
    return np.stack([occupied.astype(float), 4*mean*(1-mean), state.variance,
                     2*np.minimum(votes, 1-votes), distance, 4*local*(1-local)], axis=-1)


def fit_logistic(x, y, weights, l2=.01):
    x, y = np.asarray(x, float), np.asarray(y, float)
    weights = np.asarray(weights, float)
    weights = weights / weights.sum()
    centre = np.sum(weights[:, None]*x, axis=0)
    scale = np.sqrt(np.sum(weights[:, None]*(x-centre)**2, axis=0))
    scale = np.maximum(scale, 1e-6)
    z = np.column_stack([np.ones(len(x)), (x-centre)/scale])
    def objective(beta):
        logits = z @ beta
        loss = np.sum(weights*(np.logaddexp(0, logits)-y*logits))
        loss += .5*l2*np.dot(beta[1:], beta[1:])
        grad = z.T @ (weights*(expit(logits)-y))
        grad[1:] += l2*beta[1:]
        return float(loss), grad
    prevalence = float(np.sum(weights*y))
    initial = np.zeros(z.shape[1])
    initial[0] = np.log(max(prevalence, 1e-6)/max(1-prevalence, 1e-6))
    result = minimize(objective, initial, jac=True, method="L-BFGS-B",
                      options={"maxiter": 300, "ftol": 1e-12, "gtol": 1e-8})
    if not result.success:
        raise RuntimeError("Error model optimization failed: " + result.message)
    return dict(features=FEATURES, centre=centre.tolist(), scale=scale.tolist(),
                coefficients=result.x.tolist(), l2=l2, prevalence=prevalence,
                optimizer="scipy L-BFGS-B", iterations=int(result.nit))


class ErrorRiskModel:
    def __init__(self, payload):
        if payload["features"] != FEATURES:
            raise ValueError("Risk feature schema changed")
        self.payload = payload
        self.centre = np.asarray(payload["centre"])
        self.scale = np.asarray(payload["scale"])
        self.coefficients = np.asarray(payload["coefficients"])
        if not np.isfinite(self.coefficients).all() or np.any(self.scale <= 0):
            raise ValueError("Invalid risk model")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text())["model"])

    def predict_features(self, x):
        x = np.asarray(x)
        return expit(self.coefficients[0] + ((x-self.centre)/self.scale) @ self.coefficients[1:])

    def risk(self, state):
        out = self.predict_features(feature_grid(state))
        return np.where(state.observed == UNKNOWN, out, 0.0)
