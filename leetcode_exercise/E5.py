import sys

def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    it = iter(data)
    n = int(next(it))
    W = int(next(it))

    w = []
    v = []
    for _ in range(n):
        w.append(int(next(it)))
        v.append(int(next(it)))

    # 你的核心代码
    dp = [0] * (W + 1)
    for i in range(n):
        for j in range(W, w[i]-1,-1):
            dp[j]=max(dp[j],dp[j-w[i]]+v[i])
    # 输出结果
    print (dp[W])
if __name__ == "__main__":
    solve()