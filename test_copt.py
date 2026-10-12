import coptpy as cp
from coptpy import COPT

env = cp.Envr()
model = env.createModel("test")

x = model.addVar(lb=0, name="x")
y = model.addVar(lb=0, name="y")

model.setObjective(3 * x + 2 * y, COPT.MAXIMIZE)
model.addConstr(x + y <= 4, "c1")
model.addConstr(x + 3 * y <= 6, "c2")

model.solve()   # ← 这里改成 solve()

if model.status == COPT.OPTIMAL:
    print(f"COPT works. x={x.x:.4f}, y={y.x:.4f}, obj={model.objval:.4f}")
else:
    print(f"COPT status: {model.status}")