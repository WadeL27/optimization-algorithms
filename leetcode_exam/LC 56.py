import sys
def solve():
    data=sys.stdin.read().split()
    n_intervals=int(data[0])
    interval=[]
    for i in range(n_intervals):
        a=int(data[2*i+1])
        b=int(data[2*i+2])
        interval.append([a,b])
    interval.sort()
    merged=[]
    for l,r in interval:
        if not merged or merged[-1][-1]<l:
            merged.append([l,r])
        else:
            merged[-1][-1]=max(merged[-1][-1],r)
    for l,r in merged:
        print(l,r,sep=" ")





if __name__=="__main__":
    solve()