#!/usr/bin/env python3
import numpy as np


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

N = 12
D = 0.95

num_samples = 500

mean_sequence = [np.zeros(2) for _ in range(N)]

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
        u = np.clip(u, -2, 2)
        
        du = u - prev_u
        direction_cost = np.dot(pos_error, vel)        

        cost = error.T @ Q @ error + u.T @ R @ u + du.T @ Rd @ du + 0.5 * direction_cost
        total_cost += (gamma ** t) * cost

        u_limit = 2.0
        v_limit = 5.0

        vel_violation = np.maximum(0, np.abs(x_sim[2:]) - v_limit)
        total_cost += 500 * np.sum(vel_violation**2)

        control_violation = np.maximum(0, np.abs(u) - u_limit)
        total_cost += 500 * np.sum(control_violation**2)

        x_limit = 20.0
        y_limit = 20.0

        x_sim = A @ x_sim + B @ u

        x_sim[2:]  *= D

        if abs(x_sim[0]) > x_limit or abs(x_sim[1]) > y_limit:
            total_cost += 3000

        prev_u = u

    final_error = x_sim - target
    terminal_weight = 3.0
    total_cost += terminal_weight * (final_error.T @ Q @ final_error)
    
    return total_cost
    

def compute_cost(u_sequence):
    return simulate(x, u_sequence)


def compute_gradient(u_sequence):
    grad = np.zeros_like(u_sequence)
    eps = 1e-3

    base_cost = compute_cost(u_sequence)

    for t in range(N):
        for i in range(2):
            u_perturbed = u_sequence.copy()
            u_perturbed[t, i] += eps

            cost = compute_cost(u_perturbed)

            grad[t, i] = (cost - base_cost) / eps

    return grad


for step in range(100):

    u_sequence = np.zeros((N, 2))

    for _ in range(10):
        grad = compute_gradient(u_sequence)

        u_sequence = u_sequence - 0.1 * grad
        u_sequence = np.clip(u_sequence, -2, 2)

    best_u = u_sequence[0]

    x = A @ x + B @ best_u
    x[2:] *= D

    u_prev = best_u

    print(x[:2])