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
    import copy
    import time
    from vrptw.data.solomon_20 import get_instance_n
    from vrptw.heuristic.two_opt import improve_all, route_distance
    from vrptw.heuristic.relocate import relocate

    # ---- Config ----
    N_CUSTOMERS = 100
    NAME = 'r101'
    USE_RELOCATE = True     # 是否启用 relocate
    USE_2OPT = True         # 是否启用 2-opt

    instance = get_instance_n(N_CUSTOMERS, name=NAME)
    n = instance['n']
    d = instance['dist']

    print(f"Instance: {NAME.upper()}-{n - 1}")
    print(f"Customers: {n - 1}")
    print(f"Capacity:  {instance['capacity']}")
    print(f"Total demand: {instance['demand'].sum():.0f}")

    # ---- Stage 1: nearest (always run) ----
    t0 = time.time()
    r_nearest = nearest_neighbor(instance)
    t_nearest = time.time() - t0
    dist_nearest = sum(route_distance(r, d) for r in r_nearest)

    # ---- Stage 2: relocate (only if USE_RELOCATE) ----
    if USE_RELOCATE:
        t0 = time.time()
        r_relocate = relocate(copy.deepcopy(r_nearest), instance)
        t_relocate = time.time() - t0
        dist_relocate = sum(route_distance(r, d) for r in r_relocate)
    else:
        r_relocate = copy.deepcopy(r_nearest)
        t_relocate = 0.0
        dist_relocate = dist_nearest

    # ---- Stage 3: 2-opt (only if USE_2OPT) ----
    if USE_2OPT:
        t0 = time.time()
        r_2opt = improve_all(copy.deepcopy(r_relocate), instance)
        t_2opt = time.time() - t0
        dist_2opt = sum(route_distance(r, d) for r in r_2opt)
    else:
        r_2opt = copy.deepcopy(r_relocate)
        t_2opt = 0.0
        dist_2opt = dist_relocate

    # ---- Final result based on switches ----
    if USE_2OPT:
        final_routes, final_dist, final_name = r_2opt, dist_2opt, 'nearest+relocate+2opt'
    elif USE_RELOCATE:
        final_routes, final_dist, final_name = r_relocate, dist_relocate, 'nearest+relocate'
    else:
        final_routes, final_dist, final_name = r_nearest, dist_nearest, 'nearest'

    # ---- Stage-by-stage report ----
    print(f"\n=== Stage-by-stage ===")
    print(f"  nearest        : veh={len(r_nearest):2d}, dist={dist_nearest:8.2f}, time={t_nearest:.3f}s")
    if USE_RELOCATE:
        print(f"  + relocate     : veh={len(r_relocate):2d}, dist={dist_relocate:8.2f}, time={t_relocate:.3f}s")
    if USE_2OPT:
        print(f"  + 2-opt        : veh={len(r_2opt):2d}, dist={dist_2opt:8.2f}, time={t_2opt:.3f}s")

    # ---- Final result ----
    print(f"\n=== Final Result ===")
    print(f"Method:         {final_name}")
    print(f"Routes:         {len(final_routes)} vehicles used")
    print(f"Total distance: {final_dist:.2f}")
    print()
    for k, route in enumerate(final_routes, 1):
        print(f"  Vehicle {k}: {' -> '.join(str(i) for i in route)}")

    # ---- Feasibility check ----
    from vrptw.model.vrptw_milp import check_route_feasibility
    print("\nFeasibility check:")
    for k, route in enumerate(final_routes, 1):
        ok, msg = check_route_feasibility(route, instance)
        status = "OK" if ok else "FAIL"
        print(f"  Vehicle {k}: [{status}] {msg}")

    # ---- Plot: show stages that ran ----
    coords = instance['coords']
    stages_to_plot = [('nearest', r_nearest, dist_nearest)]
    if USE_RELOCATE:
        stages_to_plot.append(('+relocate', r_relocate, dist_relocate))
    if USE_2OPT:
        stages_to_plot.append(('+2opt', r_2opt, dist_2opt))

    n_plots = len(stages_to_plot)
    fig, axes = plt.subplots(1, n_plots, figsize=(8 * n_plots, 8))
    if n_plots == 1:
        axes = [axes]

    colors = plt.cm.tab20.colors
    for ax, (stage_name, routes, dist) in zip(axes, stages_to_plot):
        ax.scatter(coords[1:, 0], coords[1:, 1], c='blue', s=50, zorder=3,
                   label='customers')
        ax.scatter(coords[0, 0], coords[0, 1], c='red', s=200, marker='s',
                   zorder=4, label='depot')
        for i in range(1, n):
            ax.annotate(str(i), (coords[i, 0], coords[i, 1]),
                        xytext=(3, 3), textcoords='offset points', fontsize=7)
        for k, route in enumerate(routes):
            xs = [coords[i, 0] for i in route]
            ys = [coords[i, 1] for i in route]
            ax.plot(xs, ys, '-o', color=colors[k % 20], linewidth=1.5,
                    markersize=5, label=f'veh {k+1}')
        ax.set_title(f"{stage_name}: {len(routes)} routes, cost = {dist:.2f}", fontsize=11)
        ax.legend(fontsize=7, loc='best', ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    plt.tight_layout()
    suffix = f"_r{int(USE_RELOCATE)}_2o{int(USE_2OPT)}"
    out_name = f"heuristic_nearest_{NAME}_{n-1}{suffix}.png"
    plt.savefig(out_name, dpi=120)
    print(f"\nPlot saved: {out_name}")
    plt.show()