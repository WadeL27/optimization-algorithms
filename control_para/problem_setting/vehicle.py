"""Vehicle trajectory planning with a bicycle model.

State:   x = [x, y, θ, v]     position, heading, speed
Control: u = [a, ω]           acceleration, angular velocity

Dynamics:
    ẋ = v cos(θ)
    ẏ = v sin(θ)
    θ̇ = ω
    v̇ = a

Boundary:
    x(0) = [0, 0, 0, 0]
    x(T) = [5, 5, π/2, 0]
"""

import numpy as np


def dynamics(x, u):
    """State dynamics ẋ = f(x, u).动态系统"""
    theta, v = x[2], x[3]
    a, omega = u[0], u[1]
    return np.array([
        v * np.cos(theta),
        v * np.sin(theta),
        omega,
        a,
    ])


def default_params():
    """Return default problem parameters."""
    return {
        'T': 5.0,
        'N': 20,
        'x0': np.array([0.0, 0.0, 0.0, 0.0]),
        'xf': np.array([5.0, 5.0, np.pi / 2, 0.0]),
    }

def dynamics_jac(x, u):
    """Return (f, fx, fu) at (x, u) for the bicycle model.

    State: [x, y, θ, v]
    Control: [a, ω]
    """
    theta, v = x[2], x[3]
    a, omega = u[0], u[1]
    f = np.array([v * np.cos(theta),
                  v * np.sin(theta),
                  omega,
                  a])
    fx = np.array([
        [0.0, 0.0, -v * np.sin(theta), np.cos(theta)],
        [0.0, 0.0,  v * np.cos(theta), np.sin(theta)],
        [0.0, 0.0,  0.0,               0.0],
        [0.0, 0.0,  0.0,               0.0],
    ])
    fu = np.array([
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 1.0],
        [1.0, 0.0],
    ])
    return f, fx, fu