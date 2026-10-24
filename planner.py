"""
planner.py -- quintic minimum-jerk trajectory   (syllabus Topic 2: Euler-Lagrange min-jerk coefficients)

Derivation: minimise  integral( x'''(t)^2 dt ).  Euler-Lagrange gives  x^(6)(t) = 0,
so the optimum is a 5th-order polynomial  x(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5.
Six unknowns <-> six boundary conditions (pos, vel, acc at t=0 and t=T), so it is exactly determined:
    t=0 gives   c0 = p0,  c1 = v0,  c2 = a0/2                     (trivial)
    t=T gives a 3x3 linear system for c3, c4, c5                    (solved below)
"""
import numpy as np
from numpy.polynomial import polynomial as P


def quintic_coeffs(p0, v0, a0, p1, v1, a1, T):
    """Coefficients (6, n_axes) of the min-jerk polynomial. Inputs may be scalars or (n_axes,) arrays."""
    p0, v0, a0, p1, v1, a1 = (np.atleast_1d(np.asarray(v, float)) for v in (p0, v0, a0, p1, v1, a1))
    A = np.array([[T**3, T**4, T**5],                 # x(T)
                  [3 * T**2, 4 * T**3, 5 * T**4],     # x'(T)
                  [6 * T, 12 * T**2, 20 * T**3]])     # x''(T)
    b = np.array([p1 - (p0 + v0 * T + a0 / 2 * T**2),  # what is still missing at t=T after c0,c1,c2
                  v1 - (v0 + a0 * T),
                  a1 - a0])
    return np.vstack([p0, v0, a0 / 2, np.linalg.solve(A, b)])


class QuinticTrajectory3D:
    """A min-jerk path in 3-D. Calling it with t returns (pos, vel, acc): exactly what the controller expects."""

    def __init__(self, p0, v0, a0, p1, v1, a1, T):
        self.T, self.c = T, quintic_coeffs(p0, v0, a0, p1, v1, a1, T)

    def __call__(self, t):
        t = np.clip(t, 0.0, self.T)                    # hold the end state outside [0, T]
        return (P.polyval(t, self.c), P.polyval(t, P.polyder(self.c)), P.polyval(t, P.polyder(self.c, 2)))
