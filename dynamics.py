"""
dynamics.py -- linearised quadrotor model + RK4 integrator   (syllabus Topic 2)

State   s = [x, y, z, vx, vy, vz]          shape (6,)
Input   u = [theta, phi, thrust_accel]     shape (3,)
            theta, phi   : pitch / roll tilt angles (rad), small-angle approximation
            thrust_accel : net vertical acceleration (m/s^2); 0 means "exactly hovering"

Hover-linearised equations of motion (Euler-Lagrange model, small angles):
    x_ddot =  g * theta
    y_ddot = -g * phi
    z_ddot =  thrust_accel
"""
import numpy as np

G = 9.81  # gravity (m/s^2)


def state_derivative(s, u):
    """ds/dt of the linearised quadrotor: positions change by velocity, velocities by acceleration."""
    return np.array([s[3], s[4], s[5], G * u[0], -G * u[1], u[2]])


def rk4_step(s, u, dt):
    """One 4th-order Runge-Kutta step; the control u is held constant over dt (zero-order hold)."""
    k1 = state_derivative(s, u)
    k2 = state_derivative(s + 0.5 * dt * k1, u)
    k3 = state_derivative(s + 0.5 * dt * k2, u)
    k4 = state_derivative(s + dt * k3, u)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(s0, control_fn, t_end, dt=0.01):
    """Closed-loop run. control_fn(state, t) -> u. Returns time (N,), states (N,6), controls (N,3)."""
    t = np.arange(0, t_end + dt / 2, dt)
    S, U = np.zeros((len(t), 6)), np.zeros((len(t), 3))
    s = np.asarray(s0, float)
    for i, ti in enumerate(t):
        S[i], U[i] = s, control_fn(s, ti)
        s = rk4_step(s, U[i], dt)
    return t, S, U
