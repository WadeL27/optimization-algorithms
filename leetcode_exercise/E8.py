import sys


def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    n = int(data[0])
    nums=list(map(int,data[1:1+2*n]))

    intervals = []
    for i in range(n):
        intervals.append((nums[2 * i], nums[2 * i + 1]))

    intervals.sort()

    merged = []
    for l, r in intervals:
        if not merged or l > merged[-1][1]:
            merged.append([l, r])
        else:
            merged[-1][1] = max(merged[-1][1], r)

    for l, r in merged:
        print(l, r)









if __name__ == "__main__":
    solve()