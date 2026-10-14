import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
import time

def savings_algorithm(instance, time_limit=5.0, debug_customers=None):
    """Clarke-Wright savings for VRPTW.

    Returns list of routes (each route is [0, ..., 0]).
    """
    t0 = time.time()
    n = instance['n']
    d = instance['dist']
    q = instance['demand']
    Q = instance['capacity']

    # 1. 初始：每个客户单独一条路径
    routes = [[0, i, 0] for i in range(1, n)]

    # 2. 计算节约值
    savings = []
    for i in range(1, n):
        for j in range(i + 1, n):
            sij = d[0, i] + d[0, j] - d[i, j]
            if sij > 0:
                savings.append((sij, i, j))
    savings.sort(reverse=True)

    # 3. 依次尝试合并
    for sij, i, j in savings:
        if time.time() - t0 > time_limit:
            print(f"[savings] time limit {time_limit}s reached, stopping")
            break
        # ---- debug: only print if both i, j are in debug set ----
        trace = (debug_customers is not None
                 and i in debug_customers and j in debug_customers)
        
        ri = find_route(routes, i)
        rj = find_route(routes, j)

        if trace:
            print(f"\n=== try merge ({i}, {j}), s_ij = {sij:.2f} ===")
            print(f"  route_i = {ri}")
            print(f"  route_j = {rj}")

        if ri is None or rj is None or ri is rj:
            continue

        if route_load(ri, q) + route_load(rj, q) > Q:
            continue

        merged = try_merge(ri, rj, i, j, instance)
        if merged is not None:
            routes.remove(ri)
            routes.remove(rj)
            routes.append(merged)

    return routes

def find_route(routes, customer):
    for row in routes:
        for value in row:
            if value == customer:
                return row
    return None

def route_load(route,q):
    j=sum(q[i] for i in route if i!= 0)
    return j

def is_endpoint(route, customer):
    """Return 'first', 'last', or None."""
    customers = route[1:-1]  # remove depot
    if not customers:
        return None
    if customers[0] == customer:
        return 'first'
    if customers[-1] == customer:
        return 'last'
    return None

def try_merge(ri, rj, i, j, instance):
    pos_i = is_endpoint(ri, i)
    pos_j = is_endpoint(rj, j)
    if pos_i is None or pos_j is None:
        return None

    # 构造候选合并（可能有两个候选方向）
    candidates = []

    if pos_i == 'first' and pos_j == 'first':
        candidates.append(list(reversed(ri))[:-1] + rj[1:])   # 方向 A
        candidates.append(list(reversed(rj))[:-1] + ri[1:])   # 方向 B
    elif pos_i == 'first' and pos_j == 'last':
        candidates.append(rj[:-1] + ri[1:])                    # 唯一方向
    elif pos_i == 'last' and pos_j == 'first':
        candidates.append(ri[:-1] + rj[1:])                    # 唯一方向
    else:  # last-last
        candidates.append(ri[:-1] + list(reversed(rj))[1:])   # 方向 A
        candidates.append(list(reversed(ri))[:-1] + rj[1:])   # 方向 B

    # 依次尝试每个候选，返回第一个可行的
    for k in candidates:
        if route_load(k, instance['demand']) > instance['capacity']:
            continue
        if check_time_windows(k, instance):
            return k
    return None

def check_time_windows(route, instance):
    """Check if route is time-feasible. Return True/False."""
    d = instance['dist']
    e, l, s = instance['ready'], instance['due'], instance['service']
    
    t = 0.0  # 从车场出发时间
    for k in range(len(route) - 1):
        i, j = route[k], route[k+1]
        t += s[i] + d[i, j]
        if t > l[j]:
            return False  # 到达太晚
        t = max(t, e[j])  # 早到则等待
    return True
    

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import copy
    import time
    from vrptw.data.solomon_20 import get_instance_n
    from vrptw.heuristic.two_opt import improve_all, route_distance
    from vrptw.heuristic.relocate import relocate
    from vrptw.heuristic.local_refine import local_refine

    # ---- Config ----
    N_CUSTOMERS = 50
    NAME = 'r101'
    USE_RELOCATE = True
    USE_2OPT = True
    USE_LOCAL_REFINE = True     # 是否启用 COPT 局部精确优化
    N_ROUTES_TO_REFINE = 5      # 子问题路径数
    REFINE_TIME_LIMIT = 10      # 每个子问题 COPT 时间上限

    instance = get_instance_n(N_CUSTOMERS, name=NAME)
    n = instance['n']
    d = instance['dist']

    print(f"Instance: {NAME.upper()}-{n - 1}")
    print(f"Customers: {n - 1}")
    print(f"Capacity:  {instance['capacity']}")
    print(f"Total demand: {instance['demand'].sum():.0f}")

    # ---- Stage 1: savings ----
    t0 = time.time()
    r_savings = savings_algorithm(instance)
    t_savings = time.time() - t0
    dist_savings = sum(route_distance(r, d) for r in r_savings)

    # ---- Stage 2: relocate ----
    if USE_RELOCATE:
        t0 = time.time()
        r_relocate = relocate(copy.deepcopy(r_savings), instance)
        t_relocate = time.time() - t0
        dist_relocate = sum(route_distance(r, d) for r in r_relocate)
    else:
        r_relocate = copy.deepcopy(r_savings)
        t_relocate = 0.0
        dist_relocate = dist_savings

    # ---- Stage 3: 2-opt ----
    if USE_2OPT:
        t0 = time.time()
        r_2opt = improve_all(copy.deepcopy(r_relocate), instance)
        t_2opt = time.time() - t0
        dist_2opt = sum(route_distance(r, d) for r in r_2opt)
    else:
        r_2opt = copy.deepcopy(r_relocate)
        t_2opt = 0.0
        dist_2opt = dist_relocate

    # ---- Stage 4: local exact refinement with COPT ----
    if USE_LOCAL_REFINE:
        t0 = time.time()
        r_refined = local_refine(copy.deepcopy(r_2opt), instance,
                                 n_routes=N_ROUTES_TO_REFINE,
                                 time_limit=REFINE_TIME_LIMIT,
                                 verbose=True)
        t_refine = time.time() - t0
        dist_refined = sum(route_distance(r, d) for r in r_refined)
    else:
        r_refined = copy.deepcopy(r_2opt)
        t_refine = 0.0
        dist_refined = dist_2opt

    # ---- Determine final result ----
    if USE_LOCAL_REFINE:
        final_routes, final_dist = r_refined, dist_refined
        method_name = 'savings+relocate+2opt+COPT_refine'
    elif USE_2OPT:
        final_routes, final_dist = r_2opt, dist_2opt
        method_name = 'savings+relocate+2opt'
    elif USE_RELOCATE:
        final_routes, final_dist = r_relocate, dist_relocate
        method_name = 'savings+relocate'
    else:
        final_routes, final_dist = r_savings, dist_savings
        method_name = 'savings'

    # ---- Stage-by-stage report ----
    print(f"\n=== Stage-by-stage ===")
    print(f"  savings        : veh={len(r_savings):2d}, dist={dist_savings:8.2f}, time={t_savings:.3f}s")
    if USE_RELOCATE:
        print(f"  + relocate     : veh={len(r_relocate):2d}, dist={dist_relocate:8.2f}, time={t_relocate:.3f}s")
    if USE_2OPT:
        print(f"  + 2-opt        : veh={len(r_2opt):2d}, dist={dist_2opt:8.2f}, time={t_2opt:.3f}s")
    if USE_LOCAL_REFINE:
        print(f"  + COPT refine  : veh={len(r_refined):2d}, dist={dist_refined:8.2f}, time={t_refine:.3f}s")

    # ---- Final result ----
    print(f"\n=== Final Result ===")
    print(f"Method:         {method_name}")
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

    # ---- Plot ----
    coords = instance['coords']
    stages_to_plot = [('savings', r_savings, dist_savings)]
    if USE_RELOCATE:
        stages_to_plot.append(('+relocate', r_relocate, dist_relocate))
    if USE_2OPT:
        stages_to_plot.append(('+2opt', r_2opt, dist_2opt))
    if USE_LOCAL_REFINE:
        stages_to_plot.append(('+COPT', r_refined, dist_refined))

    n_plots = len(stages_to_plot)
    fig, axes = plt.subplots(1, n_plots, figsize=(7 * n_plots, 7))
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
        ax.set_title(f"{stage_name}: {len(routes)} routes, cost = {dist:.2f}",
                     fontsize=11)
        ax.legend(fontsize=7, loc='best', ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    plt.tight_layout()
    suffix = f"_r{int(USE_RELOCATE)}_2o{int(USE_2OPT)}_c{int(USE_LOCAL_REFINE)}"
    out_name = f"heuristic_savings_{NAME}_{n-1}{suffix}.png"
    plt.savefig(out_name, dpi=120)
    print(f"\nPlot saved: {out_name}")
    plt.show()