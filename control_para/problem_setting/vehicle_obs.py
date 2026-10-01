"""Vehicle trajectory planning with a circular obstacle.

State:   x = [x, y, θ, v]
Control: u = [a, ω]

Constraint:
    ||[x, y] - center|| ≥ r_safe  for all t

Boundary:
    x(0) = [0, 0, 0, 0]
    x(T) = [6, 6, π/2, 0]
"""

import numpy as np

CENTER = np.array([3.0, 3.0])
R_SAFE = 1.2


def dynamics(x, u):
    theta, v = x[2], x[3]
    a, omega = u[0], u[1]
    return np.array([
        v * np.cos(theta),
        v * np.sin(theta),
        omega,
        a,
    ])


def default_params():
    return {
        'T': 6.0,
        'N': 25,
        'x0': np.array([0.0, 0.0, 0.0, 0.0]),
        'xf': np.array([6.0, 6.0, np.pi / 2, 0.0]),
    }


def obstacle_clearance(x):
    """Return ||pos - center||² - r_safe² for each time step.

    Constraint satisfied when all entries ≥ 0.
    """
    pos = x[:, :2]
    return np.sum((pos - CENTER) ** 2, axis=1) - R_SAFE ** 2