"""Solomon C101 first 20 customers.

Fields: (x, y, demand, ready_time, due_date, service_time)
Index 0 is the depot.
"""

import numpy as np

# Solomon C101, first 20 customers (0 = depot)
RAW = [
    # x,   y,  dem,  ready,  due,  serv
    (40,  50,   0,     0,  1236,    0),   # 0 depot
    (45,  68,  10,   912,   967,   90),   # 1
    (45,  70,  30,   825,   870,   90),   # 2
    (42,  66,  10,    65,   146,   90),   # 3
    (42,  68,  10,   727,   782,   90),   # 4
    (42,  65,  10,    15,    67,   90),   # 5
    (40,  69,  20,   621,   702,   90),   # 6
    (40,  66,  20,   170,   225,   90),   # 7
    (38,  68,  20,   255,   324,   90),   # 8
    (38,  70,  10,   534,   605,   90),   # 9
    (35,  66,  10,   357,   410,   90),   # 10
    (35,  69,  10,   448,   505,   90),   # 11
    (25,  85,  20,   652,   721,   90),   # 12
    (22,  75,  30,    30,    92,   90),   # 13
    (22,  85,  10,   567,   620,   90),   # 14
    (20,  80,  40,   384,   429,   90),   # 15
    (20,  85,  40,   475,   528,   90),   # 16
    (18,  75,  20,    99,   148,   90),   # 17
    (15,  75,  20,   179,   254,   90),   # 18
    (15,  80,  10,   278,   345,   90),   # 19
    (30,  50,  10,    10,    73,   90),   # 20
]

CAPACITY = 200
N_VEHICLES = 5


def get_instance():
    """Return a dict with all instance data."""
    coords  = np.array([(r[0], r[1]) for r in RAW], dtype=float)
    demand  = np.array([r[2] for r in RAW], dtype=float)
    ready   = np.array([r[3] for r in RAW], dtype=float)
    due     = np.array([r[4] for r in RAW], dtype=float)
    service = np.array([r[5] for r in RAW], dtype=float)

    n = len(RAW)
    # Euclidean distance matrix
    dist = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            dist[i, j] = np.linalg.norm(coords[i] - coords[j])

    return {
        'n': n,
        'coords': coords,
        'demand': demand,
        'ready': ready,
        'due': due,
        'service': service,
        'dist': dist,
        'capacity': CAPACITY,
        'n_vehicles': N_VEHICLES,
    }