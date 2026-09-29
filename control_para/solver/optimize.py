"""Outer optimization for control parameterization."""

import numpy as np
from scipy.optimize import minimize

from .integrator import simulate


def make_objective(f, x0, xf, dt, m=1, penalty=1e4):
    """Build objective J(u) = ∫ u² dt + penalty * ||x(T) - xf||².

    Parameters
    ----------
    f : callable
        Dynamics ẋ = f(x, u).
    x0 : ndarray
        Initial state.
    xf : ndarray
        Target terminal state.
    dt : float
        Time step.
    m : int
        Control dimension. 1 for scalar control, 2 for 2D control.
    penalty : float
        Terminal penalty weight.

    Returns
    -------
    objective : callable
        Function that takes a flat 1D array of length N*m and returns J.
    """
    def objective(u_flat):
        u_nodes = u_flat if m == 1 else u_flat.reshape(-1, m)
        x = simulate(u_nodes, dt, x0, f)
        running = np.sum(u_nodes ** 2) * dt
        terminal = penalty * np.sum((x[-1] - xf) ** 2)
        return running + terminal

    return objective


def solve_parameterized(objective, u_init, method='BFGS', **options):
    """Wrapper around scipy.optimize.minimize.

    Parameters
    ----------
    objective : callable
        J(u_flat), where u_flat is a 1D array of length N*m.
    u_init : ndarray
        Initial guess, 1D array of length N*m.
    method : str
        Optimization method.
    **options
        Passed to scipy.optimize.minimize.
    """
    default_options = {'gtol': 1e-8, 'maxiter': 500}
    default_options.update(options)
    return minimize(objective, u_init, method=method, options=default_options)