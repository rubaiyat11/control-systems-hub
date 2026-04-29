#!/usr/bin/env python3
from collections import deque
import random
import math

target = [10.0,5.0]
position = [0.0,0.0]
velocity = [0.0,0.0]
target_velocity = [0.0,0.0]

kp_pos = 2.5
kp_vel = 2.5
ki_vel = 0.45
kd_vel = 1.0

kf = 0.35

integral = [0.0,0.0]
prev_error = [0.0,0.0]

dt = 0.01

max_output = 5.0
max_target_velocity = 5.0
max_velocity = 8.0
max_accel = 10


filtered_dx = 0
filtered_dy = 0
alpha = 0.65

delay_steps = 20
position_buffer = deque([[0.0, 0.0]]*delay_steps, maxlen=delay_steps)

delay_time = delay_steps * dt

actutator_delay_steps = 10
output_buffer = deque([[0.0,0.0]]*actutator_delay_steps, maxlen=actutator_delay_steps)

for i in range(1000):
    delayed_position = position_buffer[0]
    noise_x = random.uniform(-0.07, 0.07)
    noise_y = random.uniform(-0.07, 0.07)
    measured_position = [delayed_position[0] + noise_x, delayed_position[1] + noise_y]
    predicted_position = [
        measured_position[0] + velocity[0] * delay_time,
        measured_position[1] + velocity[1] * delay_time
        ]


    position_error = [
        target[0] - predicted_position[0],
        target[1] - predicted_position[1]
        ]
    
    vx = kp_pos * position_error[0]
    vy = kp_pos * position_error[1]

    speed = math.sqrt(vx**2 + vy**2)

    if (speed > max_target_velocity):
        scale = max_target_velocity / speed
        vx *= scale
        vy *= scale


    velocity_error = [
        vx - velocity[0],
        vy - velocity[1]
    ]


    integral[0] += velocity_error[0] * dt
    integral[1] += velocity_error[1] * dt

    new_integral = [
        integral[0] + velocity_error[0] * dt,
        integral[1] + velocity_error[1] * dt
    ]

    dx = (velocity_error[0] - prev_error[0]) / dt
    dy = (velocity_error[1] - prev_error[1]) / dt

    filtered_dx = alpha * dx + (1 - alpha) * filtered_dx  
    filtered_dy = alpha * dy + (1 - alpha) * filtered_dy
    
    dx = filtered_dx
    dy = filtered_dy

    feedforward = [kf * vx, kf * vy]

    output_x = (
        feedforward[0] +
        kp_vel * velocity_error[0] +
        ki_vel * new_integral[0] +
        kd_vel * dx
    )
    output_y = (
        feedforward[1] +
        kp_vel * velocity_error[1] +
        ki_vel * new_integral[1] +
        kd_vel * dy
    )

    output = [output_x, output_y]

    fx = output[0]
    fy = output[1]

    force_mag = math.sqrt(fx**2 + fy**2)

    if(force_mag > max_output):
        scale = max_output / force_mag
        fx *= scale
        fy *= scale
    elif(force_mag < max_output):
        integral[0] = new_integral[0]
        integral[1] = new_integral[1]
        
    output = [fx, fy]

    prev_error = velocity_error.copy()


    delayed_output = output_buffer[0]

    velocity[0] += delayed_output[0] * dt
    velocity[1] += delayed_output[1] * dt

    disturbance = random.uniform(-0.5, 0.5)
    velocity[0] += disturbance * dt
    velocity[1] += disturbance * dt

    
    velocity_mag = math.sqrt(velocity[0]**2 + velocity[1]**2)

    if velocity_mag > max_velocity:
        scale = max_velocity / velocity_mag
        velocity[0] *= scale
        velocity[1] *= scale
        

    position[0] += velocity[0] * dt
    position[1] += velocity[1] * dt


    position_buffer.append(position.copy())
    output_buffer.append(output.copy())

    print(position)

