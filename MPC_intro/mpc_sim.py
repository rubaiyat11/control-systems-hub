#!/usr/bin/env python3
import numpy as np
from itertools import product


dt = 0.1

x = np.array([0.0, 0.0, 0.0, 0.0])
target = np.array([10.0, 5.0, 0.0, 0.0])

A = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

B = np.array([
    [0, 0],
    [0, 0],
    [dt, 0],
    [0, dt]
])

Q = np.diag([20, 20, 5, 5])
R = np.diag([0.2, 0.2])
Rd = np.diag([0.5, 0.5])

N = 4
D = 0.95

u_candidates = [
    np.array([ax, ay])
    for ax in [-1, 0, 1]
    for ay in [-1, 0, 1]
]

u_prev = np.array([0.0, 0.0])

def simulate(x, u_sequence):
    x_sim = x.copy()
    total_cost = 0
    prev_u = u_prev.copy()

    gamma = 0.95

    for t, u in enumerate(u_sequence):
        error = x_sim - target
        pos_error = x_sim[:2] - target[:2]
        vel = x_sim[2:]
        du = u - prev_u
        direction_cost = np.dot(pos_error, vel)

        cost = error.T @ Q @ error + u.T @ R @ u + du.T @ Rd @ du + 0.5 * direction_cost
        total_cost += (gamma ** t) * cost

        x_sim = A @ x_sim + B @ u

        x_sim[2:]  *= D

        prev_u = u

    final_error = x_sim - target
    terminal_weight = 3.0
    total_cost += terminal_weight * (final_error.T @ Q @ final_error)
    
    return total_cost
    

for step in range(100):
    best_cost = float("inf")
    best_u = None

    for u_sequence in product(u_candidates, repeat=N):

        cost = simulate(x, u_sequence)

        if(cost < best_cost):
            best_cost = cost
            best_u = u_sequence[0]

    x = A @ x + B @ best_u

    x[2:] *= D

    u_prev = best_u

    print(x[:2])