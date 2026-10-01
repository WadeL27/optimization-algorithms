"""Example: vehicle obstacle avoidance with hard constraint (SLSQP)."""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

from control_para.problem_setting import vehicle_obs as vo
from control_para.solver.integrator import simulate
from control_para.solver.optimize import (
    make_objective, make_constraint, make_penalty_constraint, solve_parameterized
)



def run_hard_constraint(params, m=2, verbose=True):
    """Two-stage: BFGS unconstrained seed, then SLSQP with hard constraint."""
    T, N = params['T'], params['N']
    x0, xf = params['x0'], params['xf']
    dt = T / N

    objective = make_objective(vo.dynamics, x0, xf, dt, m=m)
    constraint = make_constraint(vo.dynamics, x0, dt, m, vo.obstacle_clearance)

    # ---- Stage 1: unconstrained BFGS to get a moving seed ----
    u_init = np.zeros(N * m)
    print("\n--- Stage 1: BFGS unconstrained ---")
    res_stage1 = solve_parameterized(
        objective, u_init, method='BFGS',
        verbose=verbose, print_every=200, tag='stage1-BFGS',
        gtol=1e-6, maxiter=2000,
    )

    x1 = simulate(res_stage1.x.reshape(N, m), dt, x0, vo.dynamics)
    print(f"[stage 1] success={res_stage1.success}  nfev={res_stage1.nfev}")
    print(f"[stage 1] terminal error = {np.linalg.norm(x1[-1] - xf):.4f}")
    print(f"[stage 1] min clearance  = {vo.obstacle_clearance(x1).min():.4f}")
    print(f"[stage 1] running cost   = {np.sum(res_stage1.x.reshape(N, m) ** 2) * dt:.4f}")

    # ---- Stage 2: SLSQP with hard constraint, seeded from stage 1 ----
    print("\n--- Stage 2: SLSQP with hard constraint ---")
    res = solve_parameterized(
        objective, res_stage1.x, method='SLSQP',
        constraints={'type': 'ineq', 'fun': constraint},
        constraint_func=constraint,
        verbose=verbose, print_every=1, tag='stage2-SLSQP',
        maxiter=500, ftol=1e-6,
    )
    return res, N, m, dt


def run_penalty(params, m=2, rho=1e4, verbose=True):
    """Solve with BFGS + quadratic penalty."""
    T, N = params['T'], params['N']
    x0, xf = params['x0'], params['xf']
    dt = T / N

    objective = make_objective(vo.dynamics, x0, xf, dt, m=m)
    penalty = make_penalty_constraint(vo.dynamics, x0, dt, m,
                                       vo.obstacle_clearance, rho=rho)
    constraint = make_constraint(vo.dynamics, x0, dt, m, vo.obstacle_clearance)

    def augmented(u_flat):
        return objective(u_flat) + penalty(u_flat)

    print("\n--- Penalty BFGS ---")
    u_init = np.zeros(N * m)
    res = solve_parameterized(
        augmented, u_init, method='BFGS',
        constraint_func=constraint,
        verbose=verbose, print_every=1000, tag='penalty-BFGS',
        gtol=1e-6, maxiter=2000,
    )
    return res, N, m, dt


def evaluate_and_plot(res, N, m, dt, params, tag):
    T = params['T']
    x0, xf = params['x0'], params['xf']
    u_opt = res.x.reshape(N, m)
    x_opt = simulate(u_opt, dt, x0, vo.dynamics)

    clearance = vo.obstacle_clearance(x_opt)
    min_clearance = clearance.min()

    print(f"\n=== {tag} ===")
    print(f"Success: {res.success}")
    print(f"Message: {res.message}")
    print(f"Function evals: {res.nfev}")
    print(f"Terminal error: {np.linalg.norm(x_opt[-1] - xf):.6f}")
    print(f"Min clearance:  {min_clearance:.6f}  (>= 0 means satisfied)")
    print(f"Running cost:   {np.sum(u_opt ** 2) * dt:.6f}")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Left: trajectory + obstacle
    theta_circle = np.linspace(0, 2 * np.pi, 200)
    obs_x = vo.CENTER[0] + vo.R_SAFE * np.cos(theta_circle)
    obs_y = vo.CENTER[1] + vo.R_SAFE * np.sin(theta_circle)
    axes[0].plot(obs_x, obs_y, 'r--', label='safety boundary')
    axes[0].fill(vo.CENTER[0] + 0.8 * np.cos(theta_circle),
                 vo.CENTER[1] + 0.8 * np.sin(theta_circle),
                 color='red', alpha=0.3, label='obstacle')
    axes[0].plot(x_opt[:, 0], x_opt[:, 1], 'o-', markersize=3,
                 label=tag)
    axes[0].plot(x0[0], x0[1], 'go', markersize=10, label='start')
    axes[0].plot(xf[0], xf[1], 'r*', markersize=15, label='target')
    axes[0].set_xlabel('x'); axes[0].set_ylabel('y')
    axes[0].set_title('Trajectory')
    axes[0].legend(); axes[0].grid(True); axes[0].axis('equal')

    # Right: clearance over time
    t_nodes = np.linspace(0, T, N + 1)
    axes[1].plot(t_nodes, clearance, 'o-', markersize=4)
    axes[1].axhline(0, color='r', linestyle='--', label='constraint limit')
    axes[1].set_xlabel('t'); axes[1].set_ylabel('||pos - center||² - r_safe²')
    axes[1].set_title('Constraint clearance')
    axes[1].legend(); axes[1].grid(True)

    plt.tight_layout()
    plt.savefig(f'day04_{tag}.png', dpi=120)
    plt.show(block=False)
    plt.pause(0.5)


def main():
    params = vo.default_params()

    # Method 1: hard constraint with SLSQP
    res_hard, N, m, dt = run_hard_constraint(params, m=2)
    evaluate_and_plot(res_hard, N, m, dt, params, 'hard_SLSQP')

    # Method 2: penalty with BFGS
    res_pen, N, m, dt = run_penalty(params, m=2, rho=1e4)
    evaluate_and_plot(res_pen, N, m, dt, params, 'penalty_BFGS')


if __name__ == '__main__':
    main()