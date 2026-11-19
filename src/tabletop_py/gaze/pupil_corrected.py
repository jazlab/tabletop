"""Session-calibrated 3D gaze using pupil-size correction and four small MLPs.

Train from an aligned 1 kHz NPZ with x (raw left_x, left_y, right_x,
right_y, left_pupil, right_pupil), y (LED world XYZ in mm), g (seconds
since pursuit onset), duration (moving seconds), valid, bad (moving-sample
exclusion mask), and ids (>0 on held-out static windows):

    python -m tabletop_py.gaze.pupil_corrected aligned.npz model.npz

Only the moving calibration is fitted. Static targets, robot poses and time
are never model inputs. Inference needs NumPy only; output is in world meters.
The resulting private weights are specific to a calibration/observer.
"""

import argparse
from pathlib import Path

import numpy as np

_WIDTHS = (4, 16, 32, 16, 3)
_SEEDS = (0, 1, 2, 3)


def _scales(std):
    # A constant predictor cannot calibrate any slope in that channel.
    return np.where(std > 1e-12, std, 1.0)


def _robust_pupil_fit(eye, pz, q):
    from sklearn.preprocessing import PolynomialFeatures

    xyz = q / (1 + q[:, 1:2] / 5).clip(0.35, 3)
    basis = PolynomialFeatures(2, include_bias=False).fit_transform(
        xyz.clip(-12, 12)
    )
    design = np.column_stack([basis, pz])
    weight = np.ones(len(q))
    for _ in range(3):
        w = weight / weight.sum()
        mu = w @ design
        sd = _scales(np.sqrt(w @ (design - mu) ** 2))
        z = (design - mu) / sd
        ym = w @ eye
        coeff = (
            np.linalg.solve(
                (z.T * w) @ z + 0.004 * np.eye(z.shape[1]),
                (z.T * w) @ (eye - ym),
            )
            / sd[:, None]
        )
        err = eye - (design @ coeff + (ym - mu @ coeff))
        cov = np.cov(err.T, aweights=weight) + 15 * np.eye(4)
        md = np.sqrt(np.einsum("ij,ij->i", err, np.linalg.solve(cov, err.T).T))
        weight = np.minimum(1.0, 2.4 / np.maximum(md, 0.001)) ** 2
    return coeff[-2:]


def _features(x, p):
    pz = (x[..., 4:] - p["pupil_mu"]) / p["pupil_sd"]
    return (x[..., :4] - pz @ p["pupil_co"] - p["eye_mu"]) / p["eye_sd"]


class PupilCorrectedDecoder:
    """Portable feed-forward decoder; .npz files contain no pickled objects."""

    def __init__(self, params):
        self._p = params
        self.session = str(params["session"])
        if int(params.get("format_version", 1)) != 1:
            raise ValueError("Unsupported pupil-corrected model format")
        shapes = dict(
            pupil_mu=(2,),
            pupil_sd=(2,),
            pupil_co=(2, 4),
            eye_mu=(4,),
            eye_sd=(4,),
            y_ref=(3,),
        )
        for seed in _SEEDS:
            for k, (a, b) in enumerate(zip(_WIDTHS, _WIDTHS[1:])):
                shapes[f"w{seed}_{k}"] = (b, a)
                shapes[f"b{seed}_{k}"] = (b,)
        for key, shape in shapes.items():
            a = params.get(key)
            if a is None or a.shape != shape or not np.isfinite(a).all():
                raise ValueError(f"Invalid or missing model parameter {key}")
        if any((params[k] <= 0).any() for k in ("eye_sd", "pupil_sd")):
            raise ValueError("Normalization scales must be positive")

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            return cls(dict(data))

    def save(self, path):
        """Write a new weight file; do not overwrite a previous calibration."""
        with Path(path).open("xb") as dst:
            np.savez_compressed(dst, **self._p)

    def predict_m(self, raw):
        """Predict rig-world meters for one six-channel reading or (N,6)."""
        raw = np.asarray(raw, dtype=float)
        if raw.ndim not in (1, 2) or raw.shape[-1] != 6:
            raise ValueError("Expected a six-channel sample or (N,6) array")
        if (
            not np.isfinite(raw).all()
            or (abs(raw[..., :4]) > 20000).any()
            or (raw[..., 4:] <= 0).any()
        ):
            raise ValueError("Invalid binocular sample or missing pupil size")
        p = self._p
        x = _features(raw, p).astype("f4")
        xyz = []
        for seed in _SEEDS:
            h = x
            for k in range(4):
                h = h @ p[f"w{seed}_{k}"].T + p[f"b{seed}_{k}"]
                if k < 3:
                    h = np.maximum(h, 0)
            xyz.append(h * 100.0 + p["y_ref"])
        return np.mean(xyz, axis=0) / 1000.0


def moving_indices(data):
    """Reproduce the full-lap calibration mask; reject overlapping labels."""
    t, valid, bad, ids = (data[k] for k in ("g", "valid", "bad", "ids"))
    duration = float(data["duration"])
    if (
        t.ndim != 1
        or not np.isfinite(t).all()
        or not np.all(np.diff(t) > 0)
        or not np.isfinite(duration)
        or duration <= 0.275
        or any(a.shape != t.shape for a in (valid, bad, ids))
        or valid.dtype != bool
        or bad.dtype != bool
    ):
        raise ValueError("Malformed time axis or sample masks")
    moving = (t >= 0.25) & (t < duration - 0.025)
    if np.any((ids > 0) & (t < duration)):
        raise ValueError("Static test holds overlap the stated moving period")
    eligible = np.flatnonzero(valid & ~bad & moving)
    return eligible[:: 4 if len(eligible) > 200000 else 2]


def fit_aligned(data, session="unspecified"):
    """Fit the full fixed recipe; data must meet the module's NPZ contract."""
    import torch
    from torch import nn

    ix = moving_indices(data)
    eye, target = data["x"], data["y"]
    n = len(data["g"])
    if eye.shape != (n, 6) or target.shape != (n, 3) or len(ix) < 32:
        raise ValueError("Expected aligned (N,6)/(N,3) and >=32 fitting rows")
    x, y = eye[ix].astype(float), target[ix].astype(float)
    if (
        not np.isfinite(x).all()
        or not np.isfinite(y).all()
        or (abs(x[:, :4]) > 20000).any()
        or (x[:, 4:] <= 0).any()
    ):
        raise ValueError("Invalid data marked eligible for moving calibration")
    pm, ps = x[:, 4:].mean(0), _scales(x[:, 4:].std(0))
    pz = (x[:, 4:] - pm) / ps
    ref = np.median(y, axis=0)
    q = (y - ref) / 100.0
    qsd = q.std(0)
    if (qsd <= 1e-12).any():
        raise ValueError("Moving calibration must span all three output axes")
    co = _robust_pupil_fit(x[:, :4], pz, q)
    corrected = x[:, :4] - pz @ co
    p = dict(
        pupil_mu=pm,
        pupil_sd=ps,
        pupil_co=co,
        y_ref=ref,
        eye_mu=corrected.mean(0),
        eye_sd=_scales(corrected.std(0)),
        format_version=np.array(1),
        session=np.array(session),
    )
    X, Y, S = (
        torch.as_tensor(a, dtype=torch.float32)
        for a in (_features(x, p), q, qsd)
    )
    # Fixed CPU initializations, deliberately isolated from the caller's RNG.
    with torch.random.fork_rng(devices=[]):
        for seed in _SEEDS:
            torch.random.default_generator.manual_seed(seed)
            layers = []
            for i, (a, b) in enumerate(zip(_WIDTHS, _WIDTHS[1:])):
                layers.append(nn.Linear(a, b))
                if i < 3:
                    layers.extend([nn.ReLU(), nn.Dropout(0.1)])
            model = nn.Sequential(*layers)
            opt = torch.optim.AdamW(
                model.parameters(), lr=0.001, weight_decay=0.01
            )
            lr = torch.optim.lr_scheduler.OneCycleLR(
                opt,
                max_lr=0.003,
                epochs=12,
                steps_per_epoch=(len(ix) + 1023) // 1024,
            )
            for _ in range(12):
                for j in torch.randperm(len(ix)).split(1024):
                    opt.zero_grad(set_to_none=True)
                    ((model(X[j]) - Y[j]) / S).square().mean().backward()
                    opt.step()
                    lr.step()
            for k, layer in enumerate(
                item for item in layers if isinstance(item, nn.Linear)
            ):
                p[f"w{seed}_{k}"] = layer.weight.detach().numpy().copy()
                p[f"b{seed}_{k}"] = layer.bias.detach().numpy().copy()
    return PupilCorrectedDecoder(p)


def main():
    import torch

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("aligned", type=Path)
    ap.add_argument("weights", type=Path, help="New private NPZ weights file")
    ap.add_argument("--session", default="unspecified")
    args = ap.parse_args()
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    if args.weights.exists():
        ap.error(f"Refusing to overwrite {args.weights}")
    with np.load(args.aligned, allow_pickle=False) as data:
        model = fit_aligned(data, args.session)
    model.save(args.weights)
    print(f"Saved pupil-corrected decoder to {args.weights}")


if __name__ == "__main__":
    main()
