"""
run_2_single_gate.py -- fly ONE gate with the full pipeline, then repeat over 100 random sensor-noise seeds.
Saves results/2_single_gate.png (top view + Kalman view + Monte Carlo histogram).
"""
import numpy as np
import matplotlib.pyplot as plt
from pipeline import *

r = run_chain(1, "predictive", seed=0)
g, S, t = r["gates"][0], r["S"], r["t"]
gate_y = np.array([g.true_state(x)[0][1] for x in t])
print(f"Demo run: miss distance at crossing = {r['errors'][0]:.3f} m  ({'CROSSED' if r['errors'][0] < TOL else 'MISS'}, tolerance {TOL} m)")

errs = np.array([run_chain(1, "predictive", sd)["errors"][0] for sd in range(100)])
print(f"Monte Carlo (100 seeds): success {100 * (errs < TOL).mean():.0f}%, median miss {np.median(errs):.3f} m, worst {errs.max():.3f} m")

fig, ax = plt.subplots(1, 3, figsize=(16, 4.3))
ax[0].plot(S[:, 0], S[:, 1], "b", label="drone"); ax[0].plot(np.full_like(t, g.center[0]), gate_y, "k:", label="gate sliding path")
ax[0].plot(*S[-1, :2], "go", ms=9, label="drone at crossing"); ax[0].plot(g.center[0], g.true_state(LEG_T)[0][1], "rx", ms=11, mew=3, label="gate centre then")
ax[0].set(title="Top view (x-y)", xlabel="x (m)", ylabel="y (m)"); ax[0].legend(fontsize=8)
ax[1].plot(t, S[:, 1], "b", label="drone y"); ax[1].plot(t, gate_y, "k", label="gate y (true)"); ax[1].axvline(T_OBS, c="gray", ls="--")
ax[1].text(T_OBS - .05, 0.6, "plan here", fontsize=8, ha="right"); ax[1].axvline(LEG_T, c="g"); ax[1].set(title="y vs time", xlabel="t (s)", ylabel="y (m)"); ax[1].legend(fontsize=8)
ax[2].hist(errs, bins=20, color="steelblue"); ax[2].axvline(TOL, c="r", ls="--", label=f"tolerance {TOL} m")
ax[2].set(title="Miss distance, 100 seeds", xlabel="miss (m)"); ax[2].legend()
fig.tight_layout(); save_fig(fig, "2_single_gate.png")
