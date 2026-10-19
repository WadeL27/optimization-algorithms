import sys


def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    n = int(data[0])
    nums = list(map(int, data[1:1 + n]))
    dp=[1]*len(nums)
    for i in range(len(nums)):
        for j in range(i):
            if nums[j]<nums[i]:
                dp[i]=max(dp[i],dp[j]+1)

    print (max(dp))




if __name__ == "__main__":
    solve()