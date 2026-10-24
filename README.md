# Gate-Crashers

> **Predicting moving gates with a Kalman filter and flying through them with a min-jerk quadrotor planner**

A simulation of a quadrotor drone navigating through a sequence of **laterally sliding gates** using a full sense → estimate → plan → control pipeline. Built for the *Motion Planning & Control (TE7864)* course at **TUHH**.

---

## Overview

The drone must pass through gates that oscillate sinusoidally along the y-axis, observed only through a **noisy position sensor**. The system solves this by:

1. **Estimating** the gate's position and velocity from noisy readings using a Kalman filter
2. **Forecasting** where the gate will be when the drone arrives
3. **Planning** a smooth min-jerk quintic trajectory to that predicted position
4. **Executing** the trajectory with a feedforward + PD controller

**Core insight:** Predicting the gate's *future* position (not reacting to where it is *now*) is what makes the difference — raising success rate from ~40% to **~83%** on random gate phases.

---

## Project Structure

```
gate-crashers/
│
├── dynamics.py          # Linearised quadrotor model + RK4 integrator
├── controller.py        # Feedforward + PD trajectory tracker
├── gate.py              # Sliding gate model + noisy sensor simulation
├── kalman.py            # Constant-velocity Kalman filter
├── planner.py           # Quintic min-jerk trajectory planner
├── pipeline.py          # Full system: sense → estimate → plan → control
│
├── run_1_components.py  # Unit tests for each building block
├── run_2_single_gate.py # Single gate demo + Monte Carlo (100 seeds)
├── run_3_three_gates.py # 3-gate chain demo + Monte Carlo (50 seeds)
├── run_4_baselines.py   # Comparison: predictive vs filtered vs naive
├── run_5_animation.py   # 3D animated flight (saves .mp4 or .gif)
├── run_all.py           # Runs all 5 scripts (~5 minutes)
│
├── results/             # Output figures and animation (auto-created)
├── docs/                # Slides and guide (Gate_Crashers.pptx / .docx)
│
├── requirements.txt
└── README.md
```

---

## System Architecture

```
Camera (noisy)
     │
     ▼
┌─────────────┐      forecast       ┌──────────────┐
│ Kalman Filter│ ─────────────────► │ Quintic       │
│ (kalman.py) │  pos + vel @ t_arr  │ Planner       │
└─────────────┘                     │ (planner.py)  │
                                    └──────┬────────┘
                                           │ trajectory
                                           ▼
                                    ┌──────────────┐     u=[θ,φ,az]    ┌───────────┐
                                    │ FF + PD       │ ────────────────► │ Quadrotor │
                                    │ Controller    │                   │ (RK4 sim) │
                                    │ (controller)  │ ◄──── state ───── │ (dynamics)│
                                    └──────────────┘                   └───────────┘
```

---

## Quickstart

### 1. Clone and install

```bash
git clone https://github.com/<your-username>/gate-crashers.git
cd gate-crashers
pip install -r requirements.txt
```

> **Optional:** Install `ffmpeg` for `.mp4` animation output. Without it, the animation saves as `.gif` automatically.

### 2. Run everything

```bash
python run_all.py
```

This runs all 5 steps (~5 minutes) and saves all figures + animation to `results/`.

### 3. Run individual steps

```bash
python run_1_components.py   # Unit tests: controller, Kalman, quintic planner
python run_2_single_gate.py  # Single gate + 100-seed Monte Carlo
python run_3_three_gates.py  # 3-gate chain + 50-seed Monte Carlo
python run_4_baselines.py    # Ablation study: predictive vs filtered vs naive
python run_5_animation.py    # 3D flight animation
```

### 4. Headless mode (no pop-up windows)

```bash
MPC_NO_SHOW=1 python run_all.py
```

---

## Technical Details

### Strategy: "Cruise then Intercept"

Each gate leg has two phases:

| Phase | Duration | What happens |
|-------|----------|-------------|
| **Cruise** | 3.3 s | Fly straight toward the gate's average position; Kalman filter observes and builds velocity estimate |
| **Intercept** | 0.7 s | Forecast gate position at crossing time; plan one quintic trajectory and fly it |

**Why 0.7 s for intercept?** A constant-velocity model diverges near the sine's turning point. 0.7 s ≈ 12% of the gate period, well within the near-linear region. Tuning sweep result:

| T_ARR | Success Rate |
|-------|-------------|
| 1.0 s | 37% |
| **0.7 s** | **83%** |
| 0.5 s | 48% |

### Modules

#### `dynamics.py` — Quadrotor Model
- Hover-linearised equations: `ẍ = g·θ`, `ÿ = -g·φ`, `z̈ = thrust`
- State: `[x, y, z, vx, vy, vz]` | Input: `[θ, φ, thrust_accel]`
- 4th-order Runge-Kutta integrator, `dt = 10 ms`

#### `kalman.py` — Constant-Velocity Kalman Filter
- Tracks gate `[x, y, z, vx, vy, vz]` from position-only measurements
- White-noise-acceleration process noise (coupled Q matrix — 3.5× better velocity RMSE vs diagonal Q)
- `forecast(t_ahead)` predicts future state without mutating the filter

#### `planner.py` — Min-Jerk Quintic Trajectory
- Minimises `∫ x‴(t)² dt` via Euler-Lagrange → 5th-order polynomial
- Exactly satisfies pos/vel/acc boundary conditions at `t=0` and `t=T`
- Solved as a 3×3 linear system for the high-order coefficients

#### `controller.py` — Feedforward + PD Controller
- `u = acc_ref + Kp·(p_ref − p) + Kd·(v_ref − v)`
- Inverts linearised model to get tilt angles
- Gains: `Kp = 8.0`, `Kd = 4.0` (from gain sweep: fastest settling, no oscillation)

#### `gate.py` — Sliding Gate
- Motion: `y(t) = y₀ + A·sin(ωt + φ)`, `period = 6 s`, `amplitude = 1 m`
- Sensor: noisy position-only reading, `σ = 0.05 m`

---

## Results

### Baseline Comparison (per-gate success %, 40 seeds × 3 gates, random phases)

| Planner | Synced phases | Random phases |
|---------|:---:|:---:|
| **Predictive** (KF pos + vel forecast) | **~98%** | **~83%** |
| Filtered (KF pos only, zero vel) | ~80% | ~60% |
| Naive (raw noisy sensor, zero vel) | ~65% | ~45% |

### Robustness
- **Gate speed:** Predictive method maintains advantage up to peak gate speeds of `~1.6 m/s`
- **Sensor noise:** Degrades gracefully up to `σ = 0.2 m`
- **3-gate chain:** Full chain success rate remains high with independent gate phases

---

## Course Connection

| Module | Syllabus Topic |
|--------|---------------|
| `planner.py` | Euler-Lagrange optimisation → min-jerk coefficients |
| `dynamics.py` | Linearised quadrotor model, discrete-time simulation |
| `controller.py` | Trajectory tracking, feedforward + PD |
| `kalman.py` | Kalman filtering, sensor fusion, state estimation |

---

## Requirements

```
numpy
matplotlib
```

> `ffmpeg` optional — for `.mp4` animation output (auto-falls-back to `.gif`)

---

## Author

**Aayush** — TUHH, Motion Planning & Control (TE7864)  
Supervised by Prof. Faulwasser

---

## License

MIT License — see [LICENSE](LICENSE)
