import sys
def solve():
    data=sys.stdin.read().split()
    n=int(data[0])
    nums=list(map(int,data[1:n+1]))
    dp=[min(nums)]*n
    dp[0]=nums[0]
    for i in range(1,n):
        dp[i]=max(dp[i-1]+nums[i],nums[i])
    print(max(dp))



if __name__=="__main__":
    solve()