import sys

def solve():
    data = sys.stdin.read().strip().split()
    if not data:
        return

    s = data[0]

    # 你的核心代码
    freq={}
    for char in s:
        if char in freq:
            freq[char]+=1
        else:
            freq[char]=1
    pos=-1
    for i,char in enumerate(s):
        if freq[char]==1:
            pos=i
            break

    # 输出结果
    print(pos)
if __name__ == "__main__":
    solve()