"""
kalman.py -- constant-velocity Kalman filter   (syllabus Topic 3: Kalman filtering / sensor fusion)

State x = [x, y, z, vx, vy, vz].  Only POSITION is measured; velocity is inferred by the filter.
"""
import numpy as np


class ConstantVelocityKF:
    def __init__(self, pos0, q=0.1, meas_std=0.05):
        """pos0: first position guess | q: process-noise strength (how wrong 'constant velocity' may be;
        the gate really accelerates since it is a sine) | meas_std: sensor noise (m)."""
        self.x = np.r_[pos0, np.zeros(3)]                      # velocity starts unknown (0)
        self.P = np.diag([0.1] * 3 + [10.0] * 3)               # big initial velocity uncertainty
        self.q, self.R = q, meas_std ** 2 * np.eye(3)
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])      # we observe position only

    @staticmethod
    def _F(dt):
        """Constant-velocity transition: pos += vel*dt, vel unchanged."""
        return np.kron([[1, dt], [0, 1]], np.eye(3))

    def _Q(self, dt):
        """Discrete white-noise-acceleration process noise. It couples position and velocity noise;
        using a plain diagonal Q here (our first attempt) made the velocity estimate ~3.5x worse."""
        return self.q * np.kron([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]], np.eye(3))

    def predict(self, dt):
        """Time update: push the estimate forward by dt, uncertainty grows."""
        F = self._F(dt)
        self.x, self.P = F @ self.x, F @ self.P @ F.T + self._Q(dt)

    def update(self, z):
        """Measurement update: blend the prediction with a new noisy reading z (shape (3,))."""
        S = self.H @ self.P @ self.H.T + self.R                # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)               # Kalman gain
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def state(self):
        """Current estimate (pos, vel)."""
        return self.x[:3].copy(), self.x[3:].copy()

    def forecast(self, t_ahead):
        """Predicted (pos, vel) t_ahead seconds from now, without changing the filter itself."""
        x = self._F(t_ahead) @ self.x
        return x[:3].copy(), x[3:].copy()
