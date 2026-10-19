import sys

def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    s = data[0]

    # 你的核心代码
    fre={}
    pos={}
    for i,cha in enumerate(s):
        if cha in fre:
            fre[cha]+=1
        else:
            fre[cha]=1
            pos[cha]=i
    min=fre[s[0]]
    min_p=pos[s[0]]
    for cha in fre:
        if fre[cha]<min:
            min=fre[cha]
            min_p=pos[cha]
    # 输出结果
    print(min_p)
if __name__ == "__main__":
    solve()