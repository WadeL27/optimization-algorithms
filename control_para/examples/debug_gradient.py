import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
from control_para.problem_setting import vehicle_obs as vo
from control_para.solver.adjoint import gradient_adjoint
from control_para.solver.optimize import make_objective


def check(name, params, dyn_jac, m):
    T, N = params['T'], params['N']
    x0, xf = params['x0'], params['xf']
    dt = T / N

    from control_para.solver.integrator import simulate

    # Extract the plain dynamics from dyn_jac
    def f(x, u):
        return dyn_jac(x, u)[0]     # ← 只取第一个返回值

    def objective(u_flat):
        u_nodes = u_flat.reshape(N, m)
        x = simulate(u_nodes, dt, x0, f)     # ← 传 f，不是 dyn_jac
        return np.sum(u_nodes ** 2) * dt + 1e4 * np.sum((x[-1] - xf) ** 2)

    np.random.seed(0)
    u_test = np.random.randn(N * m) * 0.1

    g_a = gradient_adjoint(u_test, dt, x0, xf, dyn_jac, m)

    g_fd = np.zeros_like(u_test)
    h = 1e-6
    J0 = objective(u_test)
    for j in range(len(u_test)):
        e = np.zeros_like(u_test); e[j] = h
        g_fd[j] = (objective(u_test + e) - J0) / h

    diff = np.abs(g_a - g_fd)
    rel = diff / (np.abs(g_fd) + 1e-12)
    cos = np.dot(g_a, g_fd) / (np.linalg.norm(g_a) * np.linalg.norm(g_fd))

    print(f"\n=== {name} ===")
    print(f"||g_adjoint|| = {np.linalg.norm(g_a):.6e}")
    print(f"||g_fd||      = {np.linalg.norm(g_fd):.6e}")
    print(f"max |diff|    = {diff.max():.6e}")
    print(f"max rel diff  = {rel.max():.6e}")
    print(f"cosine sim    = {cos:.10f}")



# Double integrator
# We need f_jac. Temporarily re-import if not in di.
# Assume di.dynamics_jac exists.


try:
    check("Vehicle", vo.default_params(), vo.dynamics_jac, m=2)
except AttributeError:
    print("vo.dynamics_jac not defined yet")