"""Local exact refinement using COPT on a subset of routes.

Idea: large-scale VRPTW is handled by heuristics; small sub-problems
(20-30 customers) are solved exactly with COPT.

思路：大规模用启发式，小规模子问题用 COPT 精确求解。
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

import numpy as np
from vrptw.model.vrptw_milp import solve_vrptw
from vrptw.heuristic.two_opt import route_distance


def _extract_subinstance(instance, customers, n_vehicles):
    """Build a small instance with only the given customers (plus depot)."""
    # Build mapping: old customer id -> new index (1-based for customers)
    old_to_new = {0: 0}
    for new_idx, old_idx in enumerate(customers, start=1):
        old_to_new[old_idx] = new_idx

    n_new = len(customers) + 1
    coords = np.zeros((n_new, 2))
    demand = np.zeros(n_new)
    ready = np.zeros(n_new)
    due = np.zeros(n_new)
    service = np.zeros(n_new)

    # depot
    coords[0] = instance['coords'][0]
    ready[0] = instance['ready'][0]
    due[0] = instance['due'][0]

    for old, new in old_to_new.items():
        if new == 0:
            continue
        coords[new] = instance['coords'][old]
        demand[new] = instance['demand'][old]
        ready[new] = instance['ready'][old]
        due[new] = instance['due'][old]
        service[new] = instance['service'][old]

    dist = np.zeros((n_new, n_new))
    for i in range(n_new):
        for j in range(n_new):
            dist[i, j] = np.linalg.norm(coords[i] - coords[j])

    sub = {
        'n': n_new,
        'coords': coords,
        'demand': demand,
        'ready': ready,
        'due': due,
        'service': service,
        'dist': dist,
        'capacity': instance['capacity'],
        'n_vehicles': n_vehicles,
    }
    return sub, old_to_new


def local_refine(routes, instance, n_routes=3, time_limit=10, verbose=True):
    """Pick a subset of routes, re-solve exactly with COPT, replace if better.

    Parameters
    ----------
    routes : list of list
    instance : dict
    n_routes : int
        Number of routes to re-optimize together. 3 means roughly 20-30
        customers in the sub-problem.
    time_limit : float
        COPT time limit per sub-problem (seconds).
    verbose : bool

    Returns
    -------
    routes : list of list
        Improved routes (or the same if no improvement).
    """
    if len(routes) <= 1:
        return routes

    n_routes = min(n_routes, len(routes))

    # Pick routes to refine. Use the longest ones by distance (they
    # contribute most to the total).
    subset_idx = sorted(
        range(len(routes)),
        key=lambda i: route_distance(routes[i], instance['dist']),
        reverse=True
    )[:n_routes]

    # Collect customers from the selected routes
    customers = []
    for i in subset_idx:
        customers.extend(c for c in routes[i] if c != 0)

    if not customers:
        return routes

    sub_inst, old_to_new = _extract_subinstance(instance, customers, n_routes)

    if verbose:
        print(f"  [local_refine] {len(customers)} customers across "
              f"{n_routes} routes, running COPT (time_limit={time_limit}s)...",
              flush=True)

    result = solve_vrptw(sub_inst, time_limit=time_limit, mip_gap=0.01, verbose=False)

    if result['objval'] is None:
        if verbose:
            print("  [local_refine] no feasible solution from COPT, skipping")
        return routes

    # Translate routes back to original customer ids
    new_routes = []
    for r in result['routes']:
        new_r = [0] + [customers[idx - 1] for idx in r[1:-1]] + [0]
        new_routes.append(new_r)

    # Compare distances
    d = instance['dist']
    old_dist = sum(route_distance(routes[i], d) for i in subset_idx)
    new_dist = sum(route_distance(r, d) for r in new_routes)

    if verbose:
        print(f"  [local_refine] old = {old_dist:.2f}, "
              f"new = {new_dist:.2f}, delta = {new_dist - old_dist:+.2f}")

    if new_dist < old_dist - 1e-6:
        final = [r for i, r in enumerate(routes) if i not in subset_idx]
        final.extend(new_routes)
        return final
    return routes