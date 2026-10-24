"""
run_3_three_gates.py -- chain of 3 moving gates (each gate's start state is the previous gate's end state).
Monte Carlo over 50 seeds. Saves results/3_three_gates.png.
"""
import numpy as np
import matplotlib.pyplot as plt
from pipeline import *

r = run_chain(3, "predictive", seed=0)
print("Demo run miss distances (m):", np.round(r["errors"], 3), "| min x-step:", f"{np.diff(r['S'][:, 0]).min():.4f} (>=0 means never flew backwards)")
per_gate, chain = monte_carlo(3, "predictive", range(50))
print(f"Monte Carlo (50 seeds): per-gate success {per_gate:.0f}%, full 3-gate chain success {chain:.0f}%")

S, t = r["S"], r["t"]
fig = plt.figure(figsize=(14, 5))
a3 = fig.add_subplot(121, projection="3d")
a3.plot(S[:, 0], S[:, 1], S[:, 2], "b", lw=2, label="drone")
for g, tc in zip(r["gates"], r["cross_t"]):                       # draw each gate's sliding path + its position at crossing
    ys = [g.true_state(x)[0][1] for x in t]
    a3.plot(np.full_like(t, g.center[0]), ys, np.full_like(t, g.center[2]), "k:", lw=1)
    a3.scatter(*g.true_state(tc)[0], c="r", s=60, marker="x")
a3.set(zlim=(0, 2), title="3-D flight (red x = gate centre when crossed)", xlabel="x", ylabel="y", zlabel="z"); a3.legend()
a = fig.add_subplot(122)
a.bar(range(1, 4), r["errors"], color="seagreen"); a.axhline(TOL, c="r", ls="--", label=f"tolerance {TOL} m")
a.set(title="Miss distance per gate (demo run)", xlabel="gate #", ylabel="miss (m)", xticks=[1, 2, 3]); a.legend()
fig.tight_layout(); save_fig(fig, "3_three_gates.png")
