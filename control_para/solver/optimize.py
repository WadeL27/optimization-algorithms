"""Outer optimization for control parameterization."""
import time

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



def solve_parameterized(objective, u_init, method='BFGS',
                        constraints=None, constraint_func=None,
                        verbose=True, print_every=1, tag='',
                        **options):
    """Wrapper around scipy.optimize.minimize with iteration display.

    Parameters
    ----------
    objective : callable
    u_init : ndarray
    method : str
    constraints : dict or list, optional
        Passed directly to scipy.optimize.minimize.
    constraint_func : callable, optional
        Only used for display (g_min). Not passed to optimizer.
    verbose : bool
        If True, attach a callback that prints progress.
    print_every : int
        Print every k iterations.
    tag : str
        Label for progress lines.
    **options
        Passed to scipy.optimize.minimize options.
    """
    default_options = {'gtol': 1e-8, 'maxiter': 500}
    default_options.update(options)

    kwargs = {'method': method, 'options': default_options}
    if constraints is not None:
        kwargs['constraints'] = constraints
    if verbose:
        kwargs['callback'] = make_callback(objective, constraint_func,
                                           print_every=print_every, tag=tag)

    return minimize(objective, u_init, **kwargs)



def make_constraint(f, x0, dt, m, constraint_func):
    """Build a constraint function g(u) ≥ 0 for scipy.optimize.

    Parameters
    ----------
    f : callable
        Dynamics ẋ = f(x, u).
    x0 : ndarray
        Initial state.
    dt : float
        Time step.
    m : int
        Control dimension.
    constraint_func : callable
        Takes state trajectory x (shape (N+1, n)), returns array of
        constraint values. Constraint satisfied when all entries ≥ 0.

    Returns
    -------
    constraint : callable
        Takes flat u array, returns array of constraint values.
    """
    def constraint(u_flat):
        u_nodes = u_flat.reshape(-1, m)
        x = simulate(u_nodes, dt, x0, f)
        return constraint_func(x)

    return constraint


def make_penalty_constraint(f, x0, dt, m, constraint_func, rho=1e4):
    """Build a penalty term for constraint violation.

    Returns a function that takes flat u and returns
        rho * sum(max(0, -g(x))²)
    """
    def penalty(u_flat):
        u_nodes = u_flat.reshape(-1, m)
        x = simulate(u_nodes, dt, x0, f)
        g = constraint_func(x)
        violation = np.minimum(g, 0.0)
        return rho * np.sum(violation ** 2)

    return penalty

def make_callback(objective, constraint=None, print_every=1, tag=''):
    """Build a callback that prints iteration progress (MATLAB Display='iter' style).

    Parameters
    ----------
    objective : callable
        J(u_flat).
    constraint : callable or None
        g(u_flat), returns array; used to report min constraint value.
        If None, skip constraint reporting.
    print_every : int
        Print every k iterations.
    tag : str
        Label prefixed to each line.
    """
    state = {'iter': 0, 't0': time.time(), 'J_best': float('inf')}

    def callback(xk):
        state['iter'] += 1
        if state['iter'] % print_every != 0:
            return

        J = objective(xk)
        if J < state['J_best']:
            state['J_best'] = J

        g_min_str = ''
        if constraint is not None:
            g = np.asarray(constraint(xk))
            g_min_str = f' | g_min = {g.min():+.3e}'

        elapsed = time.time() - state['t0']
        prefix = f'[{tag}] ' if tag else ''
        print(f'{prefix}iter {state["iter"]:4d} | J = {J:.4e} | '
              f'J_best = {state["J_best"]:.4e}{g_min_str} | t = {elapsed:.1f}s')

    return callback