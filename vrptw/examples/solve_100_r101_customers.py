"""Solve the 100-customer VRPTW with COPT."""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import matplotlib.pyplot as plt
from vrptw.data.solomon_20 import get_instance_n
from vrptw.model.vrptw_milp import solve_vrptw


def main():
    instance = get_instance_n(100,'r101')
    print(f"Customers: {instance['n'] - 1}")
    print(f"Vehicles:  {instance['n_vehicles']}")
    print(f"Capacity:  {instance['capacity']}")
    print(f"Total demand: {instance['demand'].sum():.0f}")

    result = solve_vrptw(instance, time_limit=120, mip_gap=0.01, verbose=True)
    if result['objval'] is None:
        print("No feasible solution found. Skipping plot.")
        return
    print(f"\n=== Result ===")
    print(f"Status:    {result['status']}")
    print(f"Objective: {result['objval']}")
    print(f"Routes: {len(result['routes'])} vehicles used")
    for k, route in enumerate(result['routes']):
        print(f"  Vehicle {k+1}: {' -> '.join(str(i) for i in route)}")

    from vrptw.model.vrptw_milp import check_route_feasibility

    print("\nFeasibility check:")
    for k, route in enumerate(result['routes']):
        ok, msg = check_route_feasibility(route, instance)
        print(f"  Vehicle {k+1}: {'OK' if ok else 'FAIL'} - {msg}")
        
    # ---- Plot ----
    coords = instance['coords']
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.scatter(coords[1:, 0], coords[1:, 1], c='blue', s=60, zorder=3,
               label='customers')
    ax.scatter(coords[0, 0], coords[0, 1], c='red', s=250, marker='s',
               zorder=4, label='depot')

    # Annotate customer indices
    for i in range(1, instance['n']):
        ax.annotate(str(i), (coords[i, 0], coords[i, 1]),
                    xytext=(3, 3), textcoords='offset points', fontsize=8)

    colors = plt.cm.tab10.colors
    for k, route in enumerate(result['routes']):
        xs = [coords[i, 0] for i in route]
        ys = [coords[i, 1] for i in route]
        ax.plot(xs, ys, '-o', color=colors[k % 10], linewidth=1.5,
                markersize=6, label=f'vehicle {k+1}')

    ax.set_title(f"VRPTW_r101 100 customers: {len(result['routes'])} routes, "
                 f"cost = {result['objval']:.2f}")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('vrptw_r101_100.png', dpi=120)
    plt.show()


if __name__ == '__main__':
    main()