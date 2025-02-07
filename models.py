import numpy as np
import control as ctrl

# Resistor-capacitor network
r_1 = 100000
r_2 = 500000
r_3 = 200000
c_1 = 0.000002
c_2 = 0.000010
A_rc = np.array([[-1/c_1 * (1/r_1 + 1/r_2), 1/(r_2*c_1)],
                 [1/(r_2*c_2), -1/c_2 * (1/r_2 + 1/r_3)]])
B_rc = np.array([[1/(r_1*c_1)], [0]])
C_rc = np.eye(2)
D_rc = np.zeros((2, 1))

sys_rc = ctrl.StateSpace(A_rc, B_rc, C_rc, D_rc)

# F1-tenth car
v = 6.5
L = 0.3302
A_f1 = np.array([[0, v], [0, 0]])
B_f1 = np.array([[0], [v/L]])
C_f1 = np.array([[1, 0]])
D_f1 = np.zeros((1, 1))

sys_f1 = ctrl.StateSpace(A_f1, B_f1, C_f1, D_f1)

# DC motor
A_dc = np.array([[-10, 1], [-0.02, -2]])
B_dc = np.array([[0], [2]])
C_dc = np.array([[1, 0]])
D_dc = np.zeros((1, 1))

sys_dc = ctrl.StateSpace(A_dc, B_dc, C_dc, D_dc)

# Car suspension system
A_cs = np.array([[0., 1., 0., 0.],
                 [-8., -4., 8., 4.],
                 [0., 0., 0., 1.],
                 [80., 40., -160., -60.]])
B_cs = np.array([[0.], [80.], [20.], [-1120.]])
C_cs = np.array([[1, 0, 0, 0]])
D_cs = np.zeros((1, 1))

sys_cs = ctrl.StateSpace(A_cs, B_cs, C_cs, D_cs)

# Electronic wedge brake
A_ew = np.array([[0, 1], [8.3951e3, 0]])
B_ew = np.array([[0], [4.0451]])
C_ew = np.array([[7.9920e3, 0]])
D_ew = np.zeros((1, 1))

sys_ew = ctrl.StateSpace(A_ew, B_ew, C_ew, D_ew)

# Cruise control 1
A_c1 = np.array([[-0.05]])
B_c1 = np.array([[0.01]])
C_c1 = np.array([[1]])
D_c1 = np.zeros((1, 1))

sys_c1 = ctrl.StateSpace(A_c1, B_c1, C_c1, D_c1)

# Cruise control 2
A_cc = np.array([[0, 1, 0], [0, 0, 1], [-6.0476, -5.2856, -0.238]])
B_cc = np.array([[0], [0], [2.4767]])
C_cc = np.array([[1, 0, 0]])
D_cc = np.zeros((1, 1))

sys_cc = ctrl.StateSpace(A_cc, B_cc, C_cc, D_cc)

# motivating example-1 from RTCSA-2023
A_mo1 = np.array([[5, -2], [0.7, -1]])
B_mo1 = np.array([[2], [0.2]])
C_mo1 = np.array([[1, 0]])
D_mo1 = np.zeros((1, 1))
sys_mo1 = ctrl.StateSpace(A_mo1, B_mo1, C_mo1, D_mo1)

# motivating example-2 synthetic
A_mo2 = np.array([[1, -2], [-2, -1]])
B_mo2 = np.array([[2], [0.1]])
C_mo2 = np.array([[1, 0]])
D_mo2 = np.zeros((1, 1))
sys_mo2 = ctrl.StateSpace(A_mo2, B_mo2, C_mo2, D_mo2)



# Dictionary of systems
sys_variables = {
    'RC': sys_rc,
    'F1': sys_f1,
    'DC': sys_dc,
    'CS': sys_cs,
    'EW': sys_ew,
    'C1': sys_c1,
    'CC': sys_cc,
    'MO1': sys_mo1, 'MO2': sys_mo2
}
