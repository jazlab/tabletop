"""Temporally coloured exploration noise."""

import numpy as np


def pink_noise(n_seq, length, dim, beta=1.0, rng=np.random):
    """Unit-variance coloured noise sequences [n_seq, length, dim]
    (power spectrum ~ 1/f^beta; beta=1 is pink), by spectral shaping."""
    f = np.fft.rfftfreq(length)
    f[0] = f[1]
    scale = f ** (-beta / 2)
    spec = (rng.normal(size=(n_seq, dim, len(f)))
            + 1j * rng.normal(size=(n_seq, dim, len(f)))) * scale
    x = np.fft.irfft(spec, n=length, axis=-1)
    x /= x.std(axis=-1, keepdims=True) + 1e-8
    return x.transpose(0, 2, 1).astype(np.float32)
