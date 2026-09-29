"""Generic RK4 integrator for ẋ = f(x, u) with zero-order hold."""

import numpy as np


def rk4_step(x, u, dt, f):
    """One RK4 step of ẋ = f(x, u) over interval dt."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(u_nodes, dt, x0, f):
    """Simulate over N intervals with zero-order hold on u.

    Parameters
    ----------
    u_nodes : ndarray, shape (N,) or (N, m)
        Control value at each interval, held constant within the interval.
    dt : float
        Time step.
    x0 : ndarray, shape (n,)
        Initial state.
    f : callable
        Dynamics ẋ = f(x, u).

    Returns
    -------
    x : ndarray, shape (N+1, n)
        State trajectory at nodes.求解的动态系统。
    """
    n = len(u_nodes)
    x = np.zeros((n + 1, len(x0)))
    x[0] = x0
    for i in range(n):
        x[i + 1] = rk4_step(x[i], u_nodes[i], dt, f)
    return x