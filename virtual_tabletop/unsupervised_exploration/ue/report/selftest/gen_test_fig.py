"""Generates the sample figure used by test.md."""

from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

x = np.linspace(0, 10, 200)
fig, ax = plt.subplots(figsize=(6.4, 3.2), dpi=150)
for k, c in zip(range(3), ['#2a78d6', '#d65a2a', '#52514e']):
    ax.plot(x, np.sin(x + k) * np.exp(-0.1 * k * x), color=c,
            label=f'k={k}')
ax.set_xlabel('time (s)')
ax.set_ylabel('signal')
ax.legend(frameon=False)
ax.spines[['top', 'right']].set_visible(False)
fig.tight_layout()
fig.savefig(Path(__file__).with_name('test_fig.png'))
