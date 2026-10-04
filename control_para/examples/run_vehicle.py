"""Example: vehicle trajectory planning with bicycle model."""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
import matplotlib.pyplot as plt

from control_para.problem_setting import vehicle
from control_para.solver.integrator import simulate
from control_para.solver.optimize import make_objective_finite_diff, solve_parameterized


def main():
    params = vehicle.default_params()
    T, N = params['T'], params['N']
    x0, xf = params['x0'], params['xf']
    dt = T / N
    m = 2  # control dimension

    objective,_ = make_objective_finite_diff(vehicle.dynamics, x0, xf, dt, m=m)
    u_init = np.zeros(N * m)
    res = solve_parameterized(objective, u_init, gtol=1e-6, maxiter=1000)

    print(f"Success: {res.success}")
    print(f"Message: {res.message}")
    print(f"Function evals: {res.nfev}")

    u_opt = res.x.reshape(N, m)
    x_opt = simulate(u_opt, dt, x0, vehicle.dynamics)

    J_running = np.sum(u_opt ** 2) * dt
    print(f"\nRunning cost: {J_running:.6f}")
    print(f"Final state:    {x_opt[-1]}")
    print(f"Target state:   {xf}")
    print(f"Terminal error: {np.linalg.norm(x_opt[-1] - xf):.6f}")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(x_opt[:, 0], x_opt[:, 1], 'o-', markersize=3)
    axes[0].plot(x0[0], x0[1], 'go', markersize=10, label='start')
    axes[0].plot(xf[0], xf[1], 'r*', markersize=15, label='target')
    axes[0].set_xlabel('x')
    axes[0].set_ylabel('y')
    axes[0].set_title('Vehicle trajectory')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].axis('equal')

    t_u = np.linspace(0, T, N)
    axes[1].plot(t_u, u_opt[:, 0], label='a (acceleration)')
    axes[1].plot(t_u, u_opt[:, 1], label='ω (angular velocity)')
    axes[1].set_xlabel('t')
    axes[1].set_ylabel('control')
    axes[1].set_title('Control inputs')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig('day03_vehicle.png', dpi=120)
    plt.show()


if __name__ == '__main__':
    main()