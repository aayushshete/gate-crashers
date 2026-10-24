"""
pipeline.py -- the whole Gate-Crashers system in one place: sense -> estimate -> plan -> control.

Strategy per gate ("cruise, then precision intercept"):
  1. CRUISE   for T_OBS s : fly straight toward the gate at CRUISE_V while the Kalman filter watches it.
  2. INTERCEPT for T_ARR s : plan ONE min-jerk quintic from the drone's current state to the gate's
     *predicted* state at the crossing time, then fly it with the feedforward+PD controller.
  Why not predict from t=0?  A constant-velocity model is only valid well under 1/4 of the gate's period
  (the gate is a sine wave); predicting far ahead through a direction reversal fails.  0.7 s is short enough.
  (T_ARR was swept: 1.0 s -> 37%, 0.7 s -> 83%, 0.5 s -> 48% with random gate phases; shorter loses to sensor noise.)

Three planners share the same code so the comparison is fair (see MODES).
"""
import os
import numpy as np
import matplotlib
if os.environ.get("MPC_NO_SHOW"): matplotlib.use("Agg")          # headless: save figures, don't open windows
import matplotlib.pyplot as plt

from dynamics import rk4_step
from controller import make_tracking_controller, line_trajectory
from gate import SlidingGate
from kalman import ConstantVelocityKF
from planner import QuinticTrajectory3D

# ---------------- tuned constants (see guide: they come from the tuning sweeps) ----------------
DT = 0.01                 # simulation step (s)
SENSE_EVERY = 1           # camera sampled every step = 100 Hz (as in the verified Day-4 tuning)
CRUISE_V = 1.0            # forward speed along x (m/s)
T_OBS, T_ARR = 3.3, 0.7   # observe time / intercept time (s); T_ARR/period = 0.12 (< 1/4 rule); x left at plan time = CRUISE_V*T_ARR
LEG_T = T_OBS + T_ARR     # time to fly one gate
GATE_SPACING = 4.0        # gates at x = 4, 8, 12 ...
GATE_Z = 1.0              # gate centre height
TOL = 0.3                 # SUCCESS: |drone - gate centre| < 0.3 m at the crossing instant

MODES = {                 # what the planner is told about the gate
    "predictive": "KF position + KF velocity, forecast to crossing time   (OUR METHOD)",
    "filtered":   "KF position only, zero velocity                       (ablation: no prediction)",
    "naive":      "last raw noisy measurement, zero velocity             (baseline: no filter)",
}


def fly_leg(s0, gate, t0, mode="predictive"):
    """Fly one gate. s0: drone state at leg start; t0: global time at leg start.
    Returns states S (N+1, 6) and the miss distance (m) at the crossing time t0+LEG_T."""
    kf = ConstantVelocityKF(gate.measure(t0))
    centre = gate.center
    n_obs, n_tot = round(T_OBS / DT), round(LEG_T / DT)
    # cruise: straight line to the gate's *average* position (re-centres lateral drift from the previous gate)
    v_cruise = (centre - s0[:3]) / T_OBS
    v_cruise[0] = CRUISE_V
    ctrl = make_tracking_controller(line_trajectory(s0[:3], v_cruise))
    s, S = np.asarray(s0, float), [np.asarray(s0, float)]
    for k in range(n_tot):
        t = k * DT
        if k % SENSE_EVERY == 0 and k > 0 and k <= n_obs:       # Kalman predict + update at sensor rate
            kf.predict(SENSE_EVERY * DT)
            kf.update(gate.measure(t0 + t))
        if k == n_obs:                                           # ---- switch: plan the intercept ----
            if mode == "predictive":   p1, v1 = kf.forecast(T_ARR)
            elif mode == "filtered":   p1, v1 = kf.state()[0], np.zeros(3)
            else:                      p1, v1 = gate.measure(t0 + t), np.zeros(3)
            v1 = v1.copy(); v1[0] = CRUISE_V                     # gate never moves in x: keep flying forward
            traj = QuinticTrajectory3D(s[:3], s[3:], np.zeros(3), p1, v1, np.zeros(3), T_ARR)
            ctrl = make_tracking_controller(lambda tt, tr=traj, ts=t: tr(tt - ts))
        s = rk4_step(s, ctrl(s, t), DT)
        S.append(s)
    S = np.array(S)
    return S, np.linalg.norm(S[-1, :3] - gate.true_state(t0 + LEG_T)[0])


def make_gates(n, seed, period=6.0, noise=0.05, amp=1.0, phase="sync"):
    """n gates in a row.  phase="sync": gate i is shifted by -w*i*LEG_T so that when the drone reaches it,
    it is at the same point of its sine cycle as gate 0 (otherwise leg time vs gate period drifts) -- the
    DESIGN condition.  phase="random": every gate starts at a random point of its cycle -- the stress test."""
    w, rng = 2 * np.pi / period, np.random.default_rng(seed + 999)
    return [SlidingGate([GATE_SPACING * (i + 1), 0, GATE_Z], amp, period, noise, seed * 100 + i,
                        phase=(-w * i * LEG_T) % (2 * np.pi) if phase == "sync" else rng.uniform(0, 2 * np.pi))
            for i in range(n)]


def run_chain(n_gates=3, mode="predictive", seed=0, **gate_kw):
    """Fly the drone through n_gates in a row. Returns dict: gates, t, S (full trajectory), errors, cross_t."""
    gates, s = make_gates(n_gates, seed, **gate_kw), np.array([0, 0, GATE_Z, 0, 0, 0.0])
    S, errs = [s[None]], []
    for i, g in enumerate(gates):
        leg, e = fly_leg(s, g, i * LEG_T, mode)
        S.append(leg[1:]); errs.append(e); s = leg[-1]
    S = np.vstack(S)
    return dict(gates=gates, S=S, t=np.arange(len(S)) * DT, errors=np.array(errs),
                cross_t=np.arange(1, n_gates + 1) * LEG_T)


def monte_carlo(n_gates, mode, seeds, **kw):
    """Success rates over many random sensor-noise seeds: (per-gate %, whole-chain %)."""
    E = np.array([run_chain(n_gates, mode, sd, **kw)["errors"] for sd in seeds])
    return 100 * (E < TOL).mean(), 100 * (E < TOL).all(axis=1).mean()


def save_fig(fig, name):
    """Save into results/ (relative to this file, works on any machine) and show unless MPC_NO_SHOW is set."""
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(d, exist_ok=True)
    fig.savefig(os.path.join(d, name), dpi=130, bbox_inches="tight")
    print(f"  saved results/{name}")
    if not os.environ.get("MPC_NO_SHOW"): plt.show()
    plt.close(fig)
