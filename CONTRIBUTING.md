# Contributing

This is a course project, but suggestions and improvements are welcome.

## How to contribute

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-idea`
3. Make your changes and test with `python run_all.py`
4. Commit with a clear message: `git commit -m "Add: description of change"`
5. Push and open a Pull Request

## Code style

- Follow the existing docstring style (module-level comment explaining the syllabus topic, then function docstrings)
- Keep each module focused on one responsibility
- Add a `check(...)` line to `run_1_components.py` for any new building block

## Ideas for extension

- [ ] Extended Kalman Filter for nonlinear gate motion
- [ ] MPC-based trajectory replanning at each step
- [ ] 3D gate orientation (not just lateral sliding)
- [ ] Real-time ROS integration
- [ ] Reinforcement learning baseline comparison
