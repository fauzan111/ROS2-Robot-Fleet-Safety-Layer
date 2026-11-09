"""Out-of-distribution scoring, so the safety layer knows when not to trust the detector."""

from __future__ import annotations

import numpy as np


def energy_score(logits: np.ndarray, temperature: float = 1.0) -> np.ndarray:
    """Energy score (Liu et al., NeurIPS 2020). Higher means more out-of-distribution.

    logits: (N, C). Returns (N,).
    """
    z = np.asarray(logits, dtype=float) / temperature
    m = z.max(axis=1, keepdims=True)
    lse = m.squeeze(1) + np.log(np.exp(z - m).sum(axis=1))  # numerically stable log-sum-exp
    return -temperature * lse
