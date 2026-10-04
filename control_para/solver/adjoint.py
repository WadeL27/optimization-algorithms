"""Adjoint method for gradient of control parameterization objective.

Problem:
    J = sum_k u_k^T u_k dt + penalty * ||x_N - xf||^2
    subject to x_{k+1} = rk4_step(x_k, u_k)

Adjoint:
    lambda_N = 2 * penalty * (x_N - xf)
    lambda_k = A_k^T @ lambda_{k+1},  k = N-1, ..., 0
    dJ/du_k  = 2 * u_k * dt + B_k^T @ lambda_{k+1}

where A_k = dg/dx_k, B_k = dg/du_k are the RK4-step Jacobians.
"""

import numpy as np


def rk4_step_with_jacobians(x, u, dt, f_jac):
    """One RK4 step with exact Jacobians A = dg/dx and B = dg/du.

    Parameters
    ----------
    x : ndarray, shape (n,)
    u : ndarray, shape (m,)
    dt : float
    f_jac : callable
        f_jac(x, u) -> (f, fx, fu).
        f  : ndarray (n,)
        fx : ndarray (n, n)
        fu : ndarray (n, m)

    Returns
    -------
    x_next : ndarray (n,)
    A : ndarray (n, n)
    B : ndarray (n, m)
    """
    u = np.atleast_1d(u)
    n = len(x)
    m = len(u)

    # ----- RK4 stages: values -----
    f1, fx1, fu1 = f_jac(x, u)
    k1 = f1

    y2 = x + 0.5 * dt * k1
    f2, fx2, fu2 = f_jac(y2, u)
    k2 = f2

    y3 = x + 0.5 * dt * k2
    f3, fx3, fu3 = f_jac(y3, u)
    k3 = f3

    y4 = x + dt * k3
    f4, fx4, fu4 = f_jac(y4, u)
    k4 = f4

    x_next = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    # ----- dk/dx via chain rule -----
    I = np.eye(n)
    dk1_dx = fx1
    dy2_dx = I + 0.5 * dt * dk1_dx
    dk2_dx = fx2 @ dy2_dx
    dy3_dx = I + 0.5 * dt * dk2_dx
    dk3_dx = fx3 @ dy3_dx
    dy4_dx = I + dt * dk3_dx
    dk4_dx = fx4 @ dy4_dx
    A = I + dt / 6.0 * (dk1_dx + 2 * dk2_dx + 2 * dk3_dx + dk4_dx)

    # ----- dk/du via chain rule -----
    dk1_du = fu1
    dy2_du = 0.5 * dt * dk1_du
    dk2_du = fx2 @ dy2_du + fu2
    dy3_du = 0.5 * dt * dk2_du
    dk3_du = fx3 @ dy3_du + fu3
    dy4_du = dt * dk3_du
    dk4_du = fx4 @ dy4_du + fu4
    B = dt / 6.0 * (dk1_du + 2 * dk2_du + 2 * dk3_du + dk4_du)

    return x_next, A, B


def gradient_adjoint(u_flat, dt, x0, xf, f_jac, m, penalty=1e4):
    """Compute dJ/du via the adjoint method.

    Parameters
    ----------
    u_flat : ndarray, shape (N*m,)
    dt : float
    x0 : ndarray, shape (n,)
    xf : ndarray, shape (n,)
    f_jac : callable
        f_jac(x, u) -> (f, fx, fu).
    m : int
        Control dimension.
    penalty : float
        Terminal penalty weight.

    Returns
    -------
    grad : ndarray, shape (N*m,)
    """
    N = len(u_flat) // m
    u_nodes = u_flat.reshape(N, m)
    n = len(x0)

    # ----- Forward pass: store x and Jacobians -----
    x = np.zeros((N + 1, n))
    x[0] = x0
    A_list = [None] * N
    B_list = [None] * N

    for k in range(N):
        x_next, A, B = rk4_step_with_jacobians(x[k], u_nodes[k], dt, f_jac)
        x[k + 1] = x_next
        A_list[k] = A
        B_list[k] = B

    # ----- Backward pass: propagate adjoint -----
    lam = np.zeros((N + 1, n))
    lam[N] = 2.0 * penalty * (x[N] - xf)

    for k in range(N - 1, -1, -1):
        lam[k] = A_list[k].T @ lam[k + 1]

    # ----- Assemble gradient -----
    grad = 2.0 * u_flat * dt
    for k in range(N):
        grad[k * m:(k + 1) * m] += B_list[k].T @ lam[k + 1]

    return grad