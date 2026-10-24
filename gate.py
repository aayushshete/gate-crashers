"""
gate.py -- a gate that slides sideways (y-axis) like a sine wave, plus a noisy sensor.

Two separate things (do not mix them up):
    true_state(t) : ground truth, what the gate REALLY does (hidden from the planner)
    measure(t)    : what a camera would report = truth + Gaussian noise (position only, no velocity)
"""
import numpy as np


class SlidingGate:
    def __init__(self, center, amplitude=1.0, period=6.0, noise_std=0.05, seed=None, phase=0.0):
        """center: gate's average [x,y,z] | amplitude: slide distance (m) | period: seconds per cycle
        noise_std: sensor noise (m) | seed: reproducible noise | phase: where in the sine cycle it starts."""
        self.center, self.amp = np.asarray(center, float), amplitude
        self.omega, self.phase, self.noise_std = 2 * np.pi / period, phase, noise_std
        self.rng = np.random.default_rng(seed)

    def true_state(self, t):
        """Ground-truth (pos, vel): y(t) = y0 + A sin(wt + phase), the gate never moves in x or z."""
        a = self.omega * t + self.phase
        return (self.center + [0, self.amp * np.sin(a), 0],
                np.array([0, self.amp * self.omega * np.cos(a), 0]))

    def measure(self, t):
        """One noisy position reading, as if a vision sensor looked at the gate at time t."""
        return self.true_state(t)[0] + self.rng.normal(0, self.noise_std, 3)
