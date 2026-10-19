import sys

def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    n = int(data[0])
    nums = list(map(int, data[1:1 + n]))

    # 你的核心代码
    nums_p=[0]+nums+[0]
    m=len(nums_p)
    max_s=0
    max_height=[]
    max_height_id=[]
    for i in range(m):
        while max_height and nums_p[i]<max_height[-1]:
            kuandu=i-max_height_id[-2]-1
            temp_s=max_height[-1]*kuandu
            if temp_s>max_s:
                max_s=temp_s
            max_height=max_height[:-1]
            max_height_id=max_height_id[:-1]
        if max_height and nums_p[i]>max_height[-1]:
            max_height_id.append(i)
            max_height.append(nums_p[i])
    # 输出结果
    if not max_s:
        print()
    else:
        print (max_s)
        
    


if __name__ == "__main__":
    solve()