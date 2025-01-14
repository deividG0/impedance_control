import numpy as np
import matplotlib.pyplot as plt
import random

# Parameters
K = 50.0  # Stiffness (Nm/rad)
B = 10.0  # Damping (Nm*s/rad)
I = 1.0   # Inertia (kg*m^2)

# Simulation parameters
dt = 0.01  # Time step (s)
T = 10.0    # Total simulation time (s)

# Initial conditions
q = 0.0    # Initial position (rad)
qd = 0.0   # Initial velocity (rad/s)
q_desired = 0.5  # Desired position (rad)

# Storage for plotting
time = []
positions = []
velocities = []
torques = []
external_forces = []

# Simulation loop
t = 0.0
tau_external = 1
# tau_external = random.uniform(10, 100)

while t <= T:
    # Generate random external torque between 10 and 20 Nm
    # tau_external = random.uniform(10, 100)

    # Impedance control law
    torque = K * (q_desired - q) + B * (0.0 - qd) + tau_external

    # Dynamics update
    qdd = torque / I  # Angular acceleration (rad/s^2)
    qd += qdd * dt    # Update velocity (rad/s)
    q += qd * dt      # Update position (rad)

    # Store data for plotting
    time.append(t)
    positions.append(q)
    velocities.append(qd)
    torques.append(torque)
    external_forces.append(tau_external)

    # Increment time
    t += dt

# Plot results
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time, positions, label='Position (rad)')
plt.axhline(y=q_desired, color='r', linestyle='--', label='Desired Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, velocities, label='Velocity (rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rad/s)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, torques, label='Control Torque (Nm)')
plt.plot(time, external_forces, label='External Torque (Nm)', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
