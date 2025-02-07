import numpy as np
import control as ctrl

def delay_lqr(sys, h, Q=None, R=None):
    """Design a discrete-time LQR controller for a system with a one-period control delay."""
    sysd_delay = augment(ctrl.c2d(sys, h, method='foh'))
    if Q is None:
        Q = np.eye(sysd_delay.A.shape[0])
    if R is None:
        R = np.eye(sysd_delay.B.shape[1])
    K, _, _ = ctrl.lqr(sysd_delay, Q, R)
    return K

def pole_place(sys, h, p=0.9):
    """Design a discrete-time pole-placement controller for a system with a one-period control delay."""
    sysd_delay = ctrl.sample_system(sys, h, method='foh')
    n = sysd_delay.A.shape[0]
    p_vec = np.concatenate(([0], np.full(n, p)))
    K = ctrl.place(sysd_delay.A, sysd_delay.B, p_vec)
    return K

def augment(sysd: ctrl.StateSpace):
    """Augment a discrete-time state-space model with a one-period control delay."""
    p = sysd.A.shape[0]
    q = sysd.B.shape[1]
    r = sysd.C.shape[0]
    A = np.block([[sysd.A, sysd.B], [np.zeros((q, p+q))]])
    B = np.block([[np.zeros((p, q))], [np.eye(q)]])
    C = np.block([sysd.C, np.zeros((r, q))])
    D = sysd.D
    Ts = sysd.dt
    return ctrl.StateSpace(A, B, C, D, Ts)
