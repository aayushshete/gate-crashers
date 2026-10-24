# Changelog

All notable changes to this project will be documented here.

## [1.0.0] - 2026-10-24

### Initial release

- Linearised quadrotor dynamics with RK4 integrator (`dynamics.py`)
- Feedforward + PD trajectory controller with tilt-angle inversion (`controller.py`)
- Sliding gate model with sinusoidal motion and noisy position sensor (`gate.py`)
- Constant-velocity Kalman filter with coupled process noise matrix (`kalman.py`)
- Quintic min-jerk trajectory planner via Euler-Lagrange (`planner.py`)
- Full sense→estimate→plan→control pipeline with "cruise then intercept" strategy (`pipeline.py`)
- Unit test suite for all components (`run_1_components.py`)
- Single gate demo + 100-seed Monte Carlo (`run_2_single_gate.py`)
- 3-gate chain demo + 50-seed Monte Carlo (`run_3_three_gates.py`)
- Ablation study: predictive vs filtered vs naive planner (`run_4_baselines.py`)
- 3D flight animation with gate rings and crossing flash (`run_5_animation.py`)

### Key tuning decisions

- T_ARR = 0.7 s: swept 0.5/0.7/1.0 s; 0.7 s gave best success (83%) on random phases
- Coupled Q matrix: 3.5× better velocity RMSE vs diagonal Q
- SENSE_EVERY = 1 (100 Hz): verified in Day-4 tuning
- KP = 8.0, KD = 4.0: gain sweep for fastest settling without oscillation
