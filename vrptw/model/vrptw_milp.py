"""VRPTW MILP model solved with COPT.

Variables:
    x[i, j, k]  binary   vehicle k goes from i to j
    t[i, k]     cont     arrival time at node i for vehicle k
    u[i, k]     cont     MTZ ordering variable

Objective:
    minimize total travel distance
"""

import coptpy as cp
from coptpy import COPT


def solve_vrptw(instance, time_limit=60, mip_gap=0.01, verbose=True):
    n = instance['n']
    K = instance['n_vehicles']
    Q = instance['capacity']
    d = instance['dist']
    q = instance['demand']
    e = instance['ready']
    l = instance['due']
    s = instance['service']

    # Big-M: should be >= max possible arrival time
    M = float(l.max())

    # ---- Create model ----
    env = cp.Envr()
    model = env.createModel("vrptw")

    if not verbose:
        model.setParam(COPT.Param.Logging, 0)
    model.setParam(COPT.Param.TimeLimit, time_limit)
    model.setParam(COPT.Param.RelGap, mip_gap)

    # ---- Decision variables ----
    # x[i, j, k]
    x = {}
    for k in range(K):
        for i in range(n):
            for j in range(n):
                if i != j:
                    x[i, j, k] = model.addVar(
                        vtype=COPT.BINARY, name=f"x_{i}_{j}_{k}")


    # y[k]: vehicle usage indicator  可用可不用
        y = {}
        for k in range(K):
            y[k] = model.addVar(vtype=COPT.BINARY, name=f"y_{k}")                


    # t[i, k]: arrival time, bounded by time window
    t = {}
    for k in range(K):
        for i in range(n):
            t[i, k] = model.addVar(
                lb=e[i], ub=l[i], vtype=COPT.CONTINUOUS, name=f"t_{i}_{k}")

    # u[i, k]: MTZ ordering, defined only for customers (1..n-1)
    u = {}
    for k in range(K):
        for i in range(1, n):
            u[i, k] = model.addVar(
                lb=1, ub=n - 1, vtype=COPT.CONTINUOUS, name=f"u_{i}_{k}")
    """
    # ---- Objective ---- 车辆成本不进入目标函数
    obj = cp.quicksum(
        d[i, j] * x[i, j, k]
        for k in range(K)
        for i in range(n)
        for j in range(n)
        if i != j
    )
    """

# ---- Objective ----车辆固定成本 
    VEHICLE_COST = 0.0

    obj = cp.quicksum(
    d[i, j] * x[i, j, k]
    for k in range(K) 
    for i in range(n) 
    for j in range(n) 
    if i != j
    ) + VEHICLE_COST * cp.quicksum(y[k] for k in range(K))

    model.setObjective(obj, COPT.MINIMIZE)

    # ---- Constraint 1: each customer visited exactly once ----
    for i in range(1, n):
        model.addConstr(
            cp.quicksum(x[i, j, k]
                        for k in range(K) for j in range(n) if j != i) == 1,
            name=f"visit_{i}")

    # ---- Constraint 2 & 3: each vehicle leaves and returns to depot ----
    for k in range(K):
        model.addConstr(
            cp.quicksum(x[0, j, k] for j in range(1, n)) <= y[k],
            name=f"depot_out_{k}")
        model.addConstr(
            cp.quicksum(x[i, 0, k] for i in range(1, n)) <= y[k],
            name=f"depot_in_{k}")

    # ---- Constraint 4: flow conservation at customer nodes ----
    for k in range(K):
        for j in range(1, n):
            model.addConstr(
                cp.quicksum(x[i, j, k] for i in range(n) if i != j) ==
                cp.quicksum(x[j, i, k] for i in range(n) if i != j),
                name=f"flow_{j}_{k}")

    # ---- Constraint 5: vehicle capacity ----
    for k in range(K):
        model.addConstr(
            cp.quicksum(q[i] * x[i, j, k]
                        for i in range(n) for j in range(n) if i != j) <= Q,
            name=f"cap_{k}")

    # ---- Constraint 6: time-window propagation ----
    # t[j,k] >= t[i,k] + s[i] + d[i,j] - M*(1 - x[i,j,k])
    for k in range(K):
        for i in range(n):
            for j in range(1, n):
                if i != j:
                    model.addConstr(
                        t[j, k] >= t[i, k] + s[i] + d[i, j]
                                   - M * (1 - x[i, j, k]),
                        name=f"tw_{i}_{j}_{k}")

    # ---- Constraint 7: MTZ subtour elimination ----
    # u[j,k] - u[i,k] + n * x[i,j,k] <= n - 1
    for k in range(K):
        for i in range(1, n):
            for j in range(1, n):
                if i != j:
                    model.addConstr(
                        u[j, k] - u[i, k] + n * x[i, j, k] <= n - 1,
                        name=f"mtz_{i}_{j}_{k}")

                

    # ---- Solve ----
    model.solve()

# ---- Extract routes ----
    result = {
        'status': model.status,
        'objval': None,
        'routes': [],
    }

    # Try to read objective regardless of status
    try:
        result['objval'] = model.objval
    except Exception:
        return result   # no feasible solution

    # Extract routes
    for k in range(K):
        route = [0]
        current = 0
        for _ in range(n):
            nxt = None
            for j in range(n):
                if j != current and (current, j, k) in x \
                        and x[current, j, k].x > 0.5:
                    nxt = j
                    break
            if nxt is None or nxt == 0:
                break
            route.append(nxt)
            current = nxt
        if len(route) > 1:
            route.append(0)
            result['routes'].append(route)

    return result

def check_route_feasibility(route, instance):
    dist = instance['dist']
    q = instance['demand']
    e = instance['ready']
    l = instance['due']
    s = instance['service']
    Q = instance['capacity']

    total_load = sum(q[i] for i in route if i != 0)
    if total_load > Q:
        return False, f"capacity exceeded: {total_load} > {Q}"

    t = 0.0
    for k in range(len(route) - 1):
        i, j = route[k], route[k + 1]
        t += s[i] + dist[i, j]
        if t > l[j]:
            return False, f"late at node {j}: {t:.1f} > {l[j]:.0f}"
        t = max(t, e[j])

    return True, f"load={total_load:.0f}, return_time={t:.1f}"