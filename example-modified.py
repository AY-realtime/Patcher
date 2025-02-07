import numpy as np
import control as ctrl

from models import sys_variables
from controllers import delay_lqr, augment

# motivating example system
# ~ sys = sys_variables['MO2']
# ~ sys = sys_variables['MO1']
# ~ sys = sys_variables['F1']
# ~ sys = sys_variables['RC']
# ~ sys = sys_variables['DC']
sys = sys_variables['EW']
# ~ sys = sys_variables['C1']
# ~ sys = sys_variables['CS']
# ~ sys = sys_variables['CC']

# Augment the system with a one-period control delay
sys_delay = augment(ctrl.c2d(sys, 0.1, method='foh'))

# Design a discrete-time LQR controller for the augmented system
K = delay_lqr(sys, 0.1)

print(f"Original system:\n{sys}\n")
print(f"Augmented system:\n{sys_delay}\n")
print(f"Controller gain:\n{K}\n")

# The initial state vector is [1, 1, 0], where the first two elements are the
# original state vector and the third element is the delayed control input.
# ~ x0 = np.asarray([1])
# ~ x0 = np.asarray([1, 1])
x0 = np.asarray([0.003, 0.003])
# ~ x0 = np.asarray([-1.0, -0.7333999780842345])
# ~ x0 = np.asarray([1.0, 0.7333999780842345])
# ~ x0 = np.asarray([1, 1, 1])
# ~ x0 = np.asarray([0.6, 0.6, 0.760841264737512])
# ~ x0 = np.asarray([1,1,1,1])
u0 = np.asarray([0])
z = [np.concatenate((x0, u0))]

# Simulate the system for 100 time steps
for i in range(20):
	# ~ u = -K @ z[-1]
	# ~ u = np.asarray([0])
	# ~ if i%2==0: 
	if (i==1 or i==5):# or i==8:
		u = np.asarray([0])
	else: u = -K @ z[-1]
	z.append(sys_delay.A @ z[-1] + sys_delay.B @ u)

print(f"Nominal trajectory:\n{z}\n")

for i in range(len(z[0])):
	print("\nNominal trajectory for dimension "+str(i)+":")
	print([k[i] for k in z])
# ~ print("Nominal trajectory for dimension 1:\n")
# ~ print([i[1] for i in z])
# ~ print("Nominal trajectory for dimension 2:\n")
# ~ print([i[2] for i in z])
