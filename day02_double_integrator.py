'''
Day 2: 双积分器最小能量控制 控制参数化
问题：
min_u J=int_0^T u(t)^2 dt
s.t. dx1=x2  dx2=u
x(0)=[0,0], x(T)=[1,0]
'''

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# --------------问题设置--------------
T=2.0
N=20
dt=T/N
x0=np.array([0,0])
xf=np.array([1,0])
Penalty=1e4

# ---------- 动态系统: dxdt = f(x, u) ----------
def f(x, u):
    """Double integrator: ẋ₁ = x₂, ẋ₂ = u."""
    return np.array([x[1], u])

#-------------通用RK四阶-----------
def rk4_step(x, u, dt, f):
    """One RK4 step of ẋ = f(x, u) over interval dt."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

#------------求解动态系统-------------
def simulate(u_nodes, dt,x0,f):
    n=len (u_nodes)
    x=np.zeros((n+1,len(x0)))
    x[0]=x0
    for i in range(n):
        x[i+1]=rk4_step(x[i], u_nodes[i], dt, f)
    return x

#-------------积分求解目标函数+罚终端约束------------
def obj(u_nodes):
    x=simulate(u_nodes,dt,x0,f)
    running=np.sum(u_nodes**2)*dt
    terminal =Penalty*np.sum((x[-1]-xf)**2)
    return running+terminal

#-------------优化------------------------------
u_init=np.zeros(N)
result=minimize(obj,u_init,method='BFGS',options={'gtol':1e-8,'maxiter': 500})

print(f"Success: {result.success}")
print(f"Message: {result.message}")
print(f"Function evals: {result.nfev}")

u_opt = result.x
x_opt = simulate(u_opt, dt, x0, f)

# ---------- 与解析解对比 ----------
t_nodes = np.linspace(0, T, N + 1)
t_u = t_nodes[:-1]

u_exact = 6 / T**2 - 12 * t_u / T**3
x1_exact = 3 * (t_nodes / T) ** 2 - 2 * (t_nodes / T) ** 3
x2_exact = 6 * t_nodes / T**2 - 6 * t_nodes**2 / T**3
J_exact = 12 / T**3

J_running = np.sum(u_opt ** 2) * dt
print(f"\nRunning cost (parameterized): {J_running:.6f}")
print(f"Running cost (analytical):    {J_exact:.6f}")
print(f"Relative error:               {abs(J_running - J_exact) / J_exact:.4%}")

# ---------- 画图 ----------
fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

axes[0].plot(t_u, u_opt, 'o-', label='parameterized', markersize=4)
axes[0].plot(t_u, u_exact, 'r--', label='analytical')
axes[0].set_ylabel('u(t)')
axes[0].legend(); axes[0].grid(True)

axes[1].plot(t_nodes, x_opt[:, 0], 'o-', label='parameterized', markersize=4)
axes[1].plot(t_nodes, x1_exact, 'r--', label='analytical')
axes[1].set_ylabel('x₁(t)')
axes[1].legend(); axes[1].grid(True)

axes[2].plot(t_nodes, x_opt[:, 1], 'o-', label='parameterized', markersize=4)
axes[2].plot(t_nodes, x2_exact, 'r--', label='analytical')
axes[2].set_ylabel('x₂(t)')
axes[2].set_xlabel('t')
axes[2].legend(); axes[2].grid(True)

plt.tight_layout()
plt.savefig('day02_double_integrator.png', dpi=120)
plt.show()
