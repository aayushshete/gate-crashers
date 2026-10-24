"""
controller.py -- feedforward + PD trajectory tracker   (syllabus Topics 1-2)

A trajectory is any function  traj_fn(t) -> (pos_ref, vel_ref, acc_ref)  (each shape (3,)).
The planner produces such a function; this controller flies it.
"""
import numpy as np
from dynamics import G

KP, KD = 8.0, 4.0            # PD gains (picked from a gain sweep: fastest settling, no oscillation)
MAX_TILT = np.deg2rad(25)    # keeps the small-angle linearisation valid


def make_tracking_controller(traj_fn, kp=KP, kd=KD, max_tilt=MAX_TILT):
    """
    Returns control(state, t) -> [theta, phi, thrust_accel].
        desired accel = acc_ref (feedforward: removes the lag a pure PD has on moving targets)
                      + kp*(pos error) + kd*(vel error)   (PD feedback: corrects what is left)
    The accel is then inverted through the linearised model (a_x = g*theta, a_y = -g*phi).
    """
    def control(s, t):
        p, v, a = traj_fn(t)
        ax, ay, az = a + kp * (p - s[:3]) + kd * (v - s[3:])
        return np.array([np.clip(ax / G, -max_tilt, max_tilt),
                         np.clip(-ay / G, -max_tilt, max_tilt), az])
    return control


def line_trajectory(start, velocity):
    """Constant-velocity straight line (velocity = 0 gives a hover point). Used for cruising."""
    start, velocity = np.asarray(start, float), np.asarray(velocity, float)
    return lambda t: (start + velocity * t, velocity, np.zeros(3))
