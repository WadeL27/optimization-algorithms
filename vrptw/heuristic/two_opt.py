"""2-opt local search for VRPTW routes."""

def route_distance(route, d):
    """Total distance of a route."""
    return sum(d[route[k], route[k+1]] for k in range(len(route) - 1))

def check_time_windows(route, instance):
    """Check time feasibility. Return True/False."""
    d = instance['dist']
    e = instance['ready']
    l = instance['due']
    s = instance['service']
    t = 0.0
    for k in range(len(route) - 1):
        i, j = route[k], route[k+1]
        t += s[i] + d[i, j]
        if t > l[j]:
            return False
        t = max(t, e[j])
    return True

def two_opt(route, instance, max_iter=100):
    """2-opt improvement on a single route."""
    d = instance['dist']
    improved = True
    it = 0
    while improved and it < max_iter:
        improved = False
        it += 1
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                if route_distance(new_route, d) < route_distance(route, d):
                    if check_time_windows(new_route, instance):
                        route = new_route
                        improved = True
    return route


def improve_all(routes, instance):
    """Apply 2-opt to every route."""
    return [two_opt(r, instance) for r in routes]