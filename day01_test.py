import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 2 * np.pi, 200)
y = np.sin(t) + 0.1 * np.random.randn(200)

plt.plot(t, y, label='noisy sin')
plt.plot(t, np.sin(t), 'r--', label='clean sin')
plt.legend()
plt.title('Day 1: NumPy + Matplotlib works')
plt.savefig('day01_plot.png')
plt.show()

print("Shape of t:", t.shape)
print("Max value:", y.max())