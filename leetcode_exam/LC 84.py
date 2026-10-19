import sys
def solve():
    data=sys.stdin.read().strip().split()
    n=int(data[0])
    nums=list(map(int,data[1:1+n]))
    nums_a=[0]+nums+[0]
    n+=2
    saved=[0]
    max_s=0
    for i,height in enumerate(nums_a):
        while height<nums_a[saved[-1]]:
            id=saved.pop()
            width=i-saved[-1]-1
            max_s=max(max_s,nums_a[id]*width)
        saved.append(i)
    print(max_s)





if __name__=="__main__":
    solve()