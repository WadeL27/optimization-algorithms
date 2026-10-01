import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
from scipy.optimize import minimize
from control_para.problem_setting import vehicle_obs as vo
from control_para.solver.integrator import simulate
from control_para.solver.optimize import (
    make_objective, make_constraint, make_callback,
)

params = vo.default_params()
T, N = params['T'], params['N']
x0, xf = params['x0'], params['xf']
dt = T / N
m = 2

objective = make_objective(vo.dynamics, x0, xf, dt, m=m)
constraint = make_constraint(vo.dynamics, x0, dt, m, vo.obstacle_clearance)

# ---- Run SLSQP from zero initial guess, with callback ----
print("=" * 60)
print("SLSQP from u = 0, with callback")
print("=" * 60)

u_init = np.zeros(N * m)
cb = make_callback(objective, constraint=constraint, print_every=1, tag='SLSQP')

res = minimize(
    objective, u_init, method='SLSQP',
    constraints={'type': 'ineq', 'fun': constraint},
    callback=cb,
    options={'maxiter': 20, 'ftol': 1e-6, 'disp': True},
)

print(f"\nResult: success={res.success}")
print(f"Message: {res.message}")
print(f"nfev = {res.nfev}, nit = {res.nit}")
print(f"||u_final|| = {np.linalg.norm(res.x):.3e}")