import sys

def solve():
    lines = sys.stdin.read().splitlines()
    if len(lines) < 2:
        return

    word1 = lines[0].strip()
    word2 = lines[1].strip()

    # 你的核心代码
    m, n = len(word1), len(word2)
    dp = [[0] * (n + 1) for _ in range(m + 1)]

    for i in range(m + 1):
        dp[i][0] = i
    for j in range(n + 1):
        dp[0][j] = j

    for i in range(1, m + 1):
        for j in range(1, n + 1):
            if word1[i - 1] == word2[j - 1]:
                dp[i][j] = dp[i - 1][j - 1]
            else:
                dp[i][j] = min(
                    dp[i - 1][j],
                    dp[i][j - 1],
                    dp[i - 1][j - 1]
                ) + 1

    print(dp[m][n])
    # 输出结果

if __name__ == "__main__":
    solve()