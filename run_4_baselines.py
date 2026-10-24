"""
run_4_baselines.py -- is the Kalman PREDICTION actually necessary?  Same pipeline, three planners (see pipeline.MODES):
    predictive = KF position + velocity forecast (ours) | filtered = KF position only | naive = raw noisy reading
Tests gate phase (synced vs random), gate speed (via period) and sensor noise.  40 seeds x 3 gates each.  Saves results/4_baselines.png
"""
import matplotlib.pyplot as plt
from pipeline import *

SEEDS, PERIODS, NOISES = range(40), [20, 12, 8, 6, 4], [0.02, 0.05, 0.1, 0.2]
rate = lambda mode, **kw: monte_carlo(3, mode, SEEDS, **kw)[0]          # per-gate success %
ph = {m: [rate(m, phase=p) for p in ("sync", "random")] for m in MODES}                       # A) gate phase
speed = {m: [rate(m, period=p, phase="random") for p in PERIODS] for m in MODES}              # B) gate speed
noise = {m: [rate(m, noise=n, phase="random") for n in NOISES] for m in MODES}               # C) sensor noise

row = lambda name, vals: print(f"{name:<12}" + "".join(f"{v:>8.0f}" for v in vals))
print("\nA) Per-gate success (%) at nominal settings (period 6 s, noise 0.05 m):")
print(f"{'':<12}{'synced':>8}{'random':>8}   (synced = design condition; random = gate starts anywhere in its cycle)")
for m in MODES: row(m, ph[m])
print("\nB) Success (%) vs gate period, random phase (smaller period = faster gate):")
print(f"{'period (s)':<12}" + "".join(f"{p:>8}" for p in PERIODS))
for m in MODES: row(m, speed[m])
print("\nC) Success (%) vs sensor noise std (m), random phase:")
print(f"{'noise (m)':<12}" + "".join(f"{n:>8}" for n in NOISES))
for m in MODES: row(m, noise[m])

fig, ax = plt.subplots(1, 2, figsize=(13, 4.3))
for m, c in zip(MODES, ["tab:green", "tab:orange", "tab:red"]):
    ax[0].plot([6.28 / p for p in PERIODS], speed[m], "o-", c=c, label=m); ax[1].plot(NOISES, noise[m], "o-", c=c, label=m)
ax[0].set(title="Success vs gate peak speed (random phase)", xlabel="gate peak speed (m/s)", ylabel="per-gate success (%)"); ax[0].legend()
ax[1].set(title="Success vs sensor noise (random phase)", xlabel="noise std (m)", ylabel="per-gate success (%)"); ax[1].legend()
fig.tight_layout(); save_fig(fig, "4_baselines.png")
