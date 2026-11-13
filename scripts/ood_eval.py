"""Compare OOD scorers for "is this camera frame degraded?" and fit the online one.

    uv run python scripts/ood_eval.py results/ood_id --out src/fleet_ml/models/ood_stats.json

In-distribution: clean simulation frames recorded by frame_recorder (split per robot in time:
first 60% train, next 20% validation, last 20% test). Out-of-distribution: the test frames
with each synthetic corruption (fleet_ml.ood.corrupt). Scorers:
  stats      Mahalanobis on 5 cheap image statistics (fleet_ml.ood.image_stats)
  deep       Mahalanobis on MobileNetV2 (ImageNet) penultimate features
  energy     energy score on the same network's ImageNet logits
Thresholds: the 95th percentile of validation scores is reported for comparison, but clean
frames drift during a mission (test frames come later, from other places), so at that level
21% of clean test frames were flagged. The deployed stats scorer is fitted on train+val with
threshold = 1.5 x the highest validation score: 0% clean test frames flagged, at the cost of
missing most blur and overexposure.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from fleet_ml.ood import CORRUPTIONS, Mahalanobis, auroc, corrupt, energy_score, image_stats


def split_frames(folder: Path):
    by_robot = defaultdict(list)
    for f in sorted(folder.glob("robot*_*.png")):
        if not f.stem.endswith("_depth"):
            by_robot[f.stem.split("_")[0]].append(f)
    train, val, test = [], [], []
    for files in by_robot.values():
        n = len(files)
        train += files[: int(0.6 * n)]
        val += files[int(0.6 * n): int(0.8 * n)]
        test += files[int(0.8 * n):]
    return train, val, test


def deep_features(images):
    import torch
    import torchvision

    net = torchvision.models.mobilenet_v2(
        weights=torchvision.models.MobileNet_V2_Weights.IMAGENET1K_V1).eval()
    mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
    feats, logits = [], []
    with torch.no_grad():
        for img in images:
            rgb = cv2.resize(img, (224, 224))[:, :, ::-1].copy()
            x = (torch.from_numpy(rgb).permute(2, 0, 1)[None].float() / 255 - mean) / std
            f = torch.nn.functional.adaptive_avg_pool2d(net.features(x), 1).flatten(1)
            feats.append(f.numpy()[0])
            logits.append(net.classifier(f).numpy()[0])
    return np.array(feats), np.array(logits)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("frames")
    ap.add_argument("--out", help="write the fitted stats scorer + threshold here")
    ap.add_argument("--no-deep", action="store_true")
    args = ap.parse_args()

    train, val, test = split_frames(Path(args.frames))
    load = lambda fs: [cv2.imread(str(f)) for f in fs]  # noqa: E731
    tr, va, te = load(train), load(val), load(test)
    rng = np.random.default_rng(0)
    corrupted = {k: [corrupt(im, k, rng) for im in te] for k in CORRUPTIONS}
    print(f"frames: train {len(tr)}, val {len(va)}, test {len(te)} (+{len(CORRUPTIONS)} "
          f"corruptions x {len(te)})")

    scorers = {}
    stats = Mahalanobis(n_components=5, shrinkage=0.05).fit([image_stats(i) for i in tr])
    scorers["stats"] = lambda ims: stats.score([image_stats(i) for i in ims])
    if not args.no_deep:
        f_tr, _ = deep_features(tr)
        deep = Mahalanobis(n_components=32, shrinkage=0.1).fit(f_tr)
        scorers["deep"] = lambda ims: deep.score(deep_features(ims)[0])
        scorers["energy"] = lambda ims: energy_score(deep_features(ims)[1])

    results = {}
    for name, score in scorers.items():
        s_val, s_te = score(va), score(te)
        thr = float(np.percentile(s_val, 95))
        row = {"clean_test_flagged": round(float((s_te > thr).mean()), 3), "threshold": thr}
        for k, ims in corrupted.items():
            s_ood = score(ims)
            row[k] = {"auroc": round(auroc(s_te, s_ood), 3),
                      "detected": round(float((s_ood > thr).mean()), 3)}
        results[name] = row
        aurocs = [row[k]["auroc"] for k in CORRUPTIONS]
        print(f"\n{name}: mean AUROC {np.mean(aurocs):.3f}, clean test frames flagged "
              f"{row['clean_test_flagged']:.0%} (threshold = val 95th pct)")
        for k in CORRUPTIONS:
            print(f"  {k:12s} AUROC {row[k]['auroc']:.3f}  detected {row[k]['detected']:.0%}")

    if args.out:
        deployed = Mahalanobis(n_components=5, shrinkage=0.05).fit(
            [image_stats(i) for i in tr + va])
        thr = 1.5 * float(deployed.score([image_stats(i) for i in va]).max())
        s_te = deployed.score([image_stats(i) for i in te])
        deployed_eval = {"threshold": thr, "clean_test_flagged": float((s_te > thr).mean())}
        for k, ims in corrupted.items():
            deployed_eval[k] = float((deployed.score([image_stats(i) for i in ims]) > thr).mean())
        print("\ndeployed (train+val, 1.5 x val max):",
              {k: round(v, 2) for k, v in deployed_eval.items()})
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = {"scorer": "mahalanobis_image_stats", "threshold": thr,
                   "model": deployed.to_dict(), "deployed": deployed_eval,
                   "comparison": results}
        out.write_text(json.dumps(payload, indent=1))
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
