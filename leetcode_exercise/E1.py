import sys


def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    n = int(data[0])
    nums = list(map(int, data[1:1 + n]))

    nums.sort()
    res = []
    m = len(nums)          # 用 m，避免覆盖输入的 n

    for i in range(m - 2):
        if i > 0 and nums[i] == nums[i - 1]:
            continue
        l, r = i + 1, m - 1
        while l < r:
            s = nums[i] + nums[l] + nums[r]
            if s == 0:
                res.append([nums[i], nums[l], nums[r]])
                l += 1
                r -= 1
                while l < r and nums[l] == nums[l - 1]:
                    l += 1
                while l < r and nums[r] == nums[r + 1]:
                    r -= 1
            elif s < 0:
                l += 1
            else:
                r -= 1

    if not res:
        print()            # 输出空行
    else:
        for tri in res:
            print(' '.join(map(str, tri)))


if __name__ == "__main__":
    solve()