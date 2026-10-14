"""Relocate local search for VRPTW.

Move one customer from one route to another if it reduces total distance
while keeping capacity and time windows feasible.
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from vrptw.heuristic.savings import check_time_windows, route_load
from vrptw.heuristic.two_opt import route_distance


def relocate(routes, instance, max_iter=50, verbose=False):
    """Relocate one customer across routes until no improvement.

    Parameters
    ----------
    routes : list of list
        Each route is [0, ..., 0].
    instance : dict
    max_iter : int
        Maximum number of improvement rounds.
    verbose : bool
        Print each accepted move.

    Returns
    -------
    routes : list of list
        Improved routes (same format).
    """
    d = instance['dist']
    q = instance['demand']
    Q = instance['capacity']

    routes = [list(r) for r in routes]   # copy to avoid mutating input
    improved = True
    it = 0

    while improved and it < max_iter:
        improved = False
        it += 1

        # Try moving customer at position pos_a in route a to position pos_b in route b
        for a in range(len(routes)):
            for pos_a in range(1, len(routes[a]) - 1):
                customer = routes[a][pos_a]

                # Build route a without the customer
                new_a = routes[a][:pos_a] + routes[a][pos_a + 1:]

                # If a becomes empty (only depot), skip — the customer would
                # need a new vehicle, not a relocate. (Rare case; keep simple.)
                if len(new_a) <= 2:
                    continue

                for b in range(len(routes)):
                    if a == b:
                        continue

                    for pos_b in range(1, len(routes[b])):
                        new_b = routes[b][:pos_b] + [customer] + routes[b][pos_b:]

                        # Capacity check
                        if route_load(new_b, q) > Q:
                            continue

                        # Distance delta
                        delta = (route_distance(new_a, d) + route_distance(new_b, d)
                                 - route_distance(routes[a], d)
                                 - route_distance(routes[b], d))
                        if delta >= -1e-6:
                            continue

                        # Time-window check
                        if not check_time_windows(new_a, instance):
                            continue
                        if not check_time_windows(new_b, instance):
                            continue

                        # Accept the move
                        if verbose:
                            print(f"  relocate {customer}: route {a+1} -> route {b+1}, "
                                  f"delta = {delta:+.2f}")
                        routes[a] = new_a
                        routes[b] = new_b
                        improved = True
                        break

                    if improved:
                        break
                if improved:
                    break
            if improved:
                break

    return routes