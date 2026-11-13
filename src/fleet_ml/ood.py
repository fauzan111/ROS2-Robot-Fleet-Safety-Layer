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


# --- detecting a degraded camera image (fog, blur, darkness, glare, dirt on the lens)


def auroc(in_scores: np.ndarray, out_scores: np.ndarray) -> float:
    """Probability that a random OOD sample scores higher than a random in-distribution one
    (ties count half). 0.5 = useless, 1.0 = perfect separation."""
    a = np.asarray(in_scores, dtype=float)
    b = np.asarray(out_scores, dtype=float)
    greater = (b[:, None] > a[None, :]).sum()
    ties = (b[:, None] == a[None, :]).sum()
    return float((greater + 0.5 * ties) / (a.size * b.size))


class Mahalanobis:
    """Distance to the in-distribution feature cloud, with PCA and shrinkage so it works with
    fewer samples than dimensions (Lee et al., NeurIPS 2018, single-class variant)."""

    def __init__(self, n_components: int = 32, shrinkage: float = 0.1) -> None:
        self.n_components = n_components
        self.shrinkage = shrinkage

    def fit(self, x: np.ndarray) -> Mahalanobis:
        x = np.asarray(x, dtype=float)
        self.mean_ = x.mean(axis=0)
        xc = x - self.mean_
        _, _, vt = np.linalg.svd(xc, full_matrices=False)
        self.components_ = vt[: min(self.n_components, vt.shape[0])]
        z = xc @ self.components_.T
        cov = np.cov(z, rowvar=False).reshape(z.shape[1], z.shape[1])
        target = np.trace(cov) / len(cov) * np.eye(len(cov))  # shrink towards scaled identity
        cov = (1 - self.shrinkage) * cov + self.shrinkage * target
        self.precision_ = np.linalg.inv(cov)
        return self

    def score(self, x: np.ndarray) -> np.ndarray:
        z = (np.atleast_2d(np.asarray(x, dtype=float)) - self.mean_) @ self.components_.T
        return np.sqrt(np.einsum("ij,jk,ik->i", z, self.precision_, z))

    def to_dict(self) -> dict:
        return {"mean": self.mean_.tolist(), "components": self.components_.tolist(),
                "precision": self.precision_.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> Mahalanobis:
        m = cls()
        m.mean_ = np.asarray(d["mean"])
        m.components_ = np.asarray(d["components"])
        m.precision_ = np.asarray(d["precision"])
        return m


def image_stats(bgr: np.ndarray) -> np.ndarray:
    """Cheap per-frame features: brightness, contrast, sharpness, edge density, colourfulness."""
    gray = bgr.astype(float).mean(axis=2)
    lap = (gray[1:-1, :-2] + gray[1:-1, 2:] + gray[:-2, 1:-1] + gray[2:, 1:-1]
           - 4 * gray[1:-1, 1:-1])
    gx, gy = np.abs(np.diff(gray, axis=1)), np.abs(np.diff(gray, axis=0))
    edges = ((gx[:-1, :] > 20) | (gy[:, :-1] > 20)).mean()
    sat = (bgr.max(axis=2).astype(float) - bgr.min(axis=2)).mean()
    return np.array([gray.mean(), gray.std(), np.log1p(lap.var()), edges, sat])


def corrupt(bgr: np.ndarray, kind: str, rng: np.random.Generator | None = None) -> np.ndarray:
    """Synthetic camera degradations for OOD evaluation (uint8 BGR in, uint8 BGR out)."""
    rng = rng or np.random.default_rng(0)
    img = bgr.astype(float)
    h, w = img.shape[:2]
    if kind == "fog":
        out = 0.35 * img + 0.65 * 200.0
    elif kind == "dark":
        out = img * 0.15
    elif kind == "overexposed":
        out = img * 2.5 + 60
    elif kind == "noise":
        out = img + rng.normal(0, 40, img.shape)
    elif kind == "blur":  # defocus / dirty lens: separable box blur, 15 px
        k = np.ones(15) / 15
        out = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 0, img)
        out = np.apply_along_axis(lambda v: np.convolve(v, k, mode="same"), 1, out)
    elif kind == "smudge":  # dirt covering ~40 % of the lens
        yy, xx = np.mgrid[0:h, 0:w]
        cy, cx = rng.uniform(0.3, 0.7) * h, rng.uniform(0.3, 0.7) * w
        m = np.clip(1.4 - np.hypot((yy - cy) / (0.45 * h), (xx - cx) / (0.45 * w)), 0, 1)
        out = img * (1 - m[..., None]) + 90.0 * m[..., None]
    else:
        raise ValueError(f"unknown corruption {kind}")
    return np.clip(out, 0, 255).astype(np.uint8)


CORRUPTIONS = ("fog", "blur", "dark", "overexposed", "noise", "smudge")
