import numpy as np
import control as ctrl

from models import sys_variables
from controllers import delay_lqr, augment

# F1-tenth car system
sys = sys_variables['F1']

# Augment the system with a one-period control delay
sys_delay = augment(ctrl.c2d(sys, 0.1, method='foh'))

# Design a discrete-time LQR controller for the augmented system
K = delay_lqr(sys, 0.1)

print(f"Original system:\n{sys}\n")
print(f"Augmented system:\n{sys_delay}\n")
print(f"Controller gain:\n{K}\n")

# The initial state vector is [1, 1, 0], where the first two elements are the
# original state vector and the third element is the delayed control input.
x0 = np.asarray([1, 1])
u0 = np.asarray([0])
z = [np.concatenate((x0, u0))]

# Simulate the system for 100 time steps
for i in range(100):
    u = -K @ z[-1]
    z.append(sys_delay.A @ z[-1] + sys_delay.B @ u)

print(f"Nominal trajectory:\n{z}\n")
