"""Nearest-neighbor construction for VRPTW."""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import time
from vrptw.heuristic.savings import check_time_windows, route_load
from vrptw.heuristic.two_opt import route_distance


def nearest_neighbor(instance, time_limit=5.0):
    """Nearest-neighbor construction for VRPTW.

    At each step, pick the closest feasible customer from the current node.
    If no customer fits, start a new route.

    Returns list of routes (each route is [0, ..., 0]).
    """
    t0 = time.time()
    n = instance['n']
    d = instance['dist']
    q = instance['demand']
    Q = instance['capacity']

    unvisited = set(range(1, n))
    routes = []

    while unvisited:
        if time.time() - t0 > time_limit:
            print(f"[nearest] time limit {time_limit}s reached, stopping")
            break

        route = [0]
        load = 0
        current = 0

        while True:
            # 找所有容量和时间窗都可行的候选
            candidates = []
            for c in unvisited:
                if load + q[c] > Q:
                    continue
                # 试插入到 route 末尾
                new_route = route + [c, 0]
                if check_time_windows(new_route, instance):
                    candidates.append(c)

            if not candidates:
                break

            # 选最近的
            #nxt = min(candidates, key=lambda c: d[current, c])

            # 选最紧急的
            nxt = min(candidates, key=lambda c: instance['due'][c])   # 先服务早截止的
            route.append(nxt)
            load += q[nxt]
            unvisited.remove(nxt)
            current = nxt

        route.append(0)
        routes.append(route)

    return routes


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from vrptw.data.solomon_20 import get_instance_n
    from vrptw.heuristic.two_opt import improve_all, route_distance
    import time

    # ---- Config ----
    N_CUSTOMERS = 25
    NAME = 'c101'
    USE_2OPT = False

    instance = get_instance_n(N_CUSTOMERS, name=NAME)
    n = instance['n']
    d = instance['dist']

    print(f"Instance: {NAME.upper()}-{n - 1}")
    print(f"Customers: {n - 1}")
    print(f"Capacity:  {instance['capacity']}")
    print(f"Total demand: {instance['demand'].sum():.0f}")

    # ---- Run nearest neighbor ----
    t0 = time.time()
    routes = nearest_neighbor(instance)
    t_construct = time.time() - t0

    # ---- Optional: 2-opt ----
    if USE_2OPT:
        t0 = time.time()
        routes = improve_all(routes, instance)
        t_2opt = time.time() - t0
    else:
        t_2opt = 0.0

    dist_final = sum(route_distance(r, d) for r in routes)

    # ---- Print result ----
    method_name = f"nearest{'+2opt' if USE_2OPT else ''}"
    print(f"\n=== Result ===")
    print(f"Method:        {method_name}")
    print(f"Routes:        {len(routes)} vehicles used")
    print(f"Total distance: {dist_final:.2f}")
    print(f"Time:          nearest={t_construct:.3f}s, 2opt={t_2opt:.3f}s")
    print()
    for k, route in enumerate(routes, 1):
        print(f"  Vehicle {k}: {' -> '.join(str(i) for i in route)}")

    # ---- Feasibility check ----
    from vrptw.model.vrptw_milp import check_route_feasibility
    print("\nFeasibility check:")
    for k, route in enumerate(routes, 1):
        ok, msg = check_route_feasibility(route, instance)
        status = "OK" if ok else "FAIL"
        print(f"  Vehicle {k}: [{status}] {msg}")

    # ---- Plot ----
    coords = instance['coords']
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.scatter(coords[1:, 0], coords[1:, 1], c='blue', s=60, zorder=3,
               label='customers')
    ax.scatter(coords[0, 0], coords[0, 1], c='red', s=250, marker='s',
               zorder=4, label='depot')

    for i in range(1, n):
        ax.annotate(str(i), (coords[i, 0], coords[i, 1]),
                    xytext=(3, 3), textcoords='offset points', fontsize=8)

    colors = plt.cm.tab10.colors
    for k, route in enumerate(routes):
        xs = [coords[i, 0] for i in route]
        ys = [coords[i, 1] for i in route]
        ax.plot(xs, ys, '-o', color=colors[k % 10], linewidth=1.5,
                markersize=6, label=f'vehicle {k+1}')

    title = f"{NAME.upper()}-{n-1} (nearest): {len(routes)} routes, cost = {dist_final:.2f}"
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_name = f"heuristic_nearest_{NAME}_{n-1}.png"
    plt.savefig(out_name, dpi=120)
    print(f"\nPlot saved: {out_name}")
    plt.show()