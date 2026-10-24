"""
run_1_components.py -- unit-test each building block on its own before trusting the whole system.
Prints PASS/FAIL per block and saves results/1_components.png.
"""
import numpy as np
import matplotlib.pyplot as plt
from dynamics import simulate
from controller import make_tracking_controller
from gate import SlidingGate
from kalman import ConstantVelocityKF
from planner import QuinticTrajectory3D
from pipeline import save_fig

fig, ax = plt.subplots(1, 3, figsize=(15, 4))
check = lambda name, ok, info: print(f"[{'PASS' if ok else 'FAIL'}] {name}: {info}")

# 1) Dynamics + controller: step from (0,0,1) to a hover point (2,1,2) -- must settle with no steady-state error
target = np.array([2.0, 1.0, 2.0])
ctrl = make_tracking_controller(lambda t: (target, np.zeros(3), np.zeros(3)))
t, S, _ = simulate([0, 0, 1, 0, 0, 0], ctrl, 8)
err = np.linalg.norm(S[-1, :3] - target)
check("controller step response", err < 0.01, f"final error {err:.4f} m")
ax[0].plot(t, S[:, :3]); ax[0].set(title="1) Controller: step to (2,1,2)", xlabel="t (s)", ylabel="x,y,z (m)")
ax[0].legend(["x", "y", "z"])

# 2) Kalman filter: track a moving gate from noisy position-only data; compare to raw sensor
gate = SlidingGate([4, 0, 1], seed=1)
kf, ts = ConstantVelocityKF(gate.measure(0)), np.arange(0.01, 6, 0.01)
est, raw, true = [], [], []
for ti in ts:
    kf.predict(0.01); z = gate.measure(ti); kf.update(z)
    est.append(np.r_[kf.state()[0][1], kf.state()[1][1]]); raw.append(z[1]); true.append([gate.true_state(ti)[0][1], gate.true_state(ti)[1][1]])
est, true, raw = np.array(est), np.array(true), np.array(raw)
rmse_p, rmse_raw, rmse_v = (np.sqrt(np.mean((est[100:, 0] - true[100:, 0]) ** 2)),
                            np.sqrt(np.mean((raw[100:] - true[100:, 0]) ** 2)), np.sqrt(np.mean((est[100:, 1] - true[100:, 1]) ** 2)))
check("Kalman filter", rmse_p < rmse_raw, f"pos RMSE {rmse_p:.3f} m (raw sensor {rmse_raw:.3f}), velocity RMSE {rmse_v:.3f} m/s")
ax[1].plot(ts, raw, ".", ms=2, c="lightgray", label="raw sensor"); ax[1].plot(ts, true[:, 0], "k", label="true")
ax[1].plot(ts, est[:, 0], "r", label="Kalman"); ax[1].set(title="2) Kalman: gate y-position", xlabel="t (s)", ylabel="y (m)"); ax[1].legend()

# 3) Quintic: must hit all six boundary conditions exactly, and be smoother than a naive cubic-like profile
p0, v0, a0, p1, v1, a1 = np.array([0, 0, 1.]), np.array([1, 0, 0.]), np.zeros(3), np.array([3, 1, 1.]), np.array([1, .5, 0]), np.zeros(3)
q = QuinticTrajectory3D(p0, v0, a0, p1, v1, a1, 2.0)
ok = all(np.allclose(a, b, atol=1e-9) for a, b in zip(q(0.0) + q(2.0), (p0, v0, a0, p1, v1, a1)))
check("quintic boundary conditions", ok, "pos/vel/acc match at t=0 and t=T")
tt = np.linspace(0, 2, 200); P = np.array([q(x)[0] for x in tt]); V = np.array([q(x)[1] for x in tt])
ax[2].plot(tt, P[:, 1], label="y pos"); ax[2].plot(tt, V[:, 1], label="y vel"); ax[2].set(title="3) Min-jerk quintic (y axis)", xlabel="t (s)"); ax[2].legend()
fig.tight_layout(); save_fig(fig, "1_components.png")
