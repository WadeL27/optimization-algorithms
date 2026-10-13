"""Parameter sensitivity analysis for R101-100 VRPTW.

Three experiments, each varying one parameter while fixing others:
  A. TimeLimit  ∈ {30, 60, 120}
  B. RelGap     ∈ {0.001, 0.01, 0.05}
  C. Threads    ∈ {1, 4, 20}
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from vrptw.data.solomon_20 import get_instance_n
from vrptw.model.vrptw_milp import solve_vrptw


def run_one(instance, time_limit, mip_gap, threads, tag):
    """Run one experiment, return a summary dict."""
    result = solve_vrptw(
        instance,
        time_limit=time_limit,
        mip_gap=mip_gap,
        threads=threads,
        verbose=False,
    )
    return {
        'tag': tag,
        'time_limit': time_limit,
        'mip_gap': mip_gap,
        'threads': threads,
        'objval': result['objval'],
        'best_bound': result['best_bound'],
        'best_gap': result['best_gap'],
        'solve_time': result['solve_time'],
        'n_vehicles': len(result['routes']),
    }


def print_table(title, rows, vary_col):
    print(f"\n{'=' * 70}")
    print(title)
    print('=' * 70)
    print(f"{vary_col:>12} | {'objval':>10} | {'best_bound':>12} | "
          f"{'gap(%)':>8} | {'time(s)':>8} | {'veh':>4}")
    print('-' * 70)
    for r in rows:
        gap_pct = r['best_gap'] * 100 if r['best_gap'] is not None else float('nan')
        print(f"{r[vary_col]:>12} | {r['objval']:>10.2f} | {r['best_bound']:>12.2f} | "
              f"{gap_pct:>8.3f} | {r['solve_time']:>8.2f} | {r['n_vehicles']:>4}")


def main():
    instance = get_instance_n(100, name='r101')
    print(f"Instance: R101-100, total demand = {instance['demand'].sum():.0f}")

    # ---- Experiment A: TimeLimit ----
    print("\n>>> Experiment A: vary TimeLimit (RelGap=0.01, default threads)")
    rows_A = []
    for tl in [30, 60, 120]:
        print(f"  running TimeLimit={tl}...", flush=True)
        rows_A.append(run_one(instance, time_limit=tl, mip_gap=0.01,
                              threads=None, tag=f'TL={tl}'))
    print_table("Experiment A: TimeLimit sensitivity", rows_A, 'time_limit')

    # ---- Experiment B: RelGap ----
    print("\n>>> Experiment B: vary RelGap (TimeLimit=120, default threads)")
    rows_B = []
    for gap in [0.001, 0.01, 0.05]:
        print(f"  running RelGap={gap}...", flush=True)
        rows_B.append(run_one(instance, time_limit=120, mip_gap=gap,
                              threads=None, tag=f'gap={gap}'))
    print_table("Experiment B: RelGap sensitivity", rows_B, 'mip_gap')

    # ---- Experiment C: Threads ----
    print("\n>>> Experiment C: vary Threads (TimeLimit=60, RelGap=0.01)")
    rows_C = []
    for th in [1, 4, 20]:
        print(f"  running Threads={th}...", flush=True)
        rows_C.append(run_one(instance, time_limit=60, mip_gap=0.01,
                              threads=th, tag=f'th={th}'))
    print_table("Experiment C: Threads sensitivity", rows_C, 'threads')


if __name__ == '__main__':
    main()