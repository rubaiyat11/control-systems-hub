#/usr/bin/env python3
import numpy as np
from scipy.linalg import solve_continuous_are
from collections import deque


#BROKE THE CONTROLLER THANKS TO EXCESS DAMPING & NON-LINEARITY


dt = 0.01

x = np.array([0.0, 0.0, 0.0, 0.0])
target = np.array([10.0, 5.0, 0.0, 0.0])

A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
])

m = 2.0
B= np.array([
    [0, 0],
    [0, 0],
    [1/m, 0],
    [0, 1/m]
])

Q = np.diag([10, 10, 1, 1])
R = np.diag([0.1, 0.1])

P = solve_continuous_are(A, B, Q, R)

K = np.linalg.inv(R) @ B.T @ P

u_buffer = deque([np.zeros(2)] * 10, maxlen=10)

for i in range(1000):
    target[0] += 0.01
    target[1] += 0.005

    error = x - target

    u = -K @ error
    u = np.clip(u, -2.0, 2.0)

    u_buffer.append(u)
    u_applied = u_buffer[0]

    c = 0.5
    A[2, 2] = -c/m
    A[3, 3] = -c/m

    disturbance = np.array([0.0, 0.0, 0.2, -0.1])

    x_dot = A @ x + B @ u_applied + disturbance
    
    drag = 0.05
    x_dot[2] -= drag * x[2] * abs(x[2])
    x_dot[3] -= drag * x[3] * abs(x[3])

    x += x_dot * dt


    print(x[:2])