"""
run_5_animation.py -- live animation of the 3-gate flight (3-D view + top view) -> results/gate_crashers.mp4 (or .gif).
Gate = ring (radius = success tolerance) sliding along y.  A green "CROSSED!" flash appears at each crossing.
Close the window to finish; set MPC_NO_SHOW=1 to only save the file.
"""
import os, numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from pipeline import *

STEP, FPS, FLASH = 4, 25, 0.25                       # 4 sim steps per frame x 25 fps = real time (dt 0.01)
r = run_chain(3, "predictive", seed=0)
S, t, gates = r["S"], r["t"], r["gates"]
ang = np.linspace(0, 2 * np.pi, 60)
fig = plt.figure(figsize=(13, 6)); a3 = fig.add_subplot(121, projection="3d"); a2 = fig.add_subplot(122)
a3.set(xlim=(0, 14), ylim=(-2, 2), zlim=(0, 2), xlabel="x", ylabel="y", zlabel="z"); a3.view_init(22, -62)
a2.set(xlim=(0, 14), ylim=(-2, 2), xlabel="x (m)", ylabel="y (m)", title="Top view"); a2.set_aspect("equal")
rings = [a3.plot([], [], [], "k", lw=3)[0] for _ in gates]; bars = [a2.plot([], [], "k", lw=4)[0] for _ in gates]
trail3, = a3.plot([], [], [], "b"); trail2, = a2.plot([], [], "b"); dot3, = a3.plot([], [], [], "ro", ms=8); dot2, = a2.plot([], [], "ro")
flash = fig.text(0.5, 0.96, "", ha="center", fontsize=22, weight="bold", color="green"); clock = fig.text(0.02, 0.96, "", fontsize=11)

def draw(f):
    k = min(f * STEP, len(t) - 1); tk = t[k]
    for g, ring, bar in zip(gates, rings, bars):
        cx, cy, cz = g.true_state(tk)[0]
        ring.set_data_3d(np.full(60, cx), cy + TOL * np.cos(ang), cz + TOL * np.sin(ang))
        bar.set_data([cx, cx], [cy - TOL, cy + TOL])
    trail3.set_data_3d(S[:k, 0], S[:k, 1], S[:k, 2]); trail2.set_data(S[:k, 0], S[:k, 1])
    dot3.set_data_3d([S[k, 0]], [S[k, 1]], [S[k, 2]]); dot2.set_data([S[k, 0]], [S[k, 1]])
    hit = [i for i, tc in enumerate(r["cross_t"]) if 0 <= tk - tc < FLASH]
    flash.set_text(f"CROSSED gate {hit[0] + 1}!  miss {r['errors'][hit[0]] * 100:.0f} cm" if hit else "")
    clock.set_text(f"t = {tk:.2f} s"); a3.set_title("Gate-Crashers", y=0.92)

anim = animation.FuncAnimation(fig, draw, frames=len(t) // STEP + 1, interval=1000 / FPS)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "gate_crashers")
os.makedirs(os.path.dirname(out), exist_ok=True)
for f in (int(r["cross_t"][1] / DT) // STEP + 3,):    # still frame at the 2nd crossing, for the slides/guide
    draw(f); fig.savefig(out + "_frame.png", dpi=110)
try:
    anim.save(out + ".mp4", writer=animation.FFMpegWriter(fps=FPS, bitrate=2500)); print("saved results/gate_crashers.mp4")
except Exception:                                      # no ffmpeg installed -> animated GIF instead
    anim.save(out + ".gif", writer=animation.PillowWriter(fps=FPS)); print("ffmpeg not found -> saved results/gate_crashers.gif")
if not os.environ.get("MPC_NO_SHOW"): plt.show()
