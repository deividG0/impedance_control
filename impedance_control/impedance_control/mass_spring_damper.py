import numpy as np

# Define parameters
def inertia_tensor(q):
    # Example inertia tensor for a 3-DOF manipulator
    M = np.diag([1.0 + 0.1*q[0]**2, 1.0 + 0.1*q[1]**2, 1.0 + 0.1*q[2]**2])
    return M

def coriolis_matrix(q, q_dot):
    # Example Coriolis matrix for a 3-DOF manipulator
    C = np.zeros((3, 3))
    C[0, 1] = 0.1 * q_dot[1]
    C[1, 2] = 0.1 * q_dot[2]
    return C

def gravity_torques(q):
    # Example gravity torques for a 3-DOF manipulator
    g = 9.81  # gravity
    G = np.array([g * np.sin(q[0]), g * np.sin(q[1]), g * np.sin(q[2])])
    return G

def jacobian_matrix(q):
    # Example Jacobian for a 3-DOF manipulator
    J = np.array([[np.cos(q[0]), 0, 0],
                  [0, np.cos(q[1]), 0],
                  [0, 0, np.cos(q[2])]])
    return J

# Simulation parameters
dt = 0.01  # time step
num_steps = 1000  # number of simulation steps

# Initial conditions
q = np.array([0.1, 0.1, 0.1])  # initial joint positions
q_dot = np.array([0.0, 0.0, 0.0])  # initial joint velocities

# External forces (example)
f_ext = np.array([1.0, 0.0, -1.0])

# Control torques
tau_act = np.array([0.0, 0.0, 0.0])

# Mass-spring-damper parameters
k = np.array([50.0, 50.0, 50.0])  # spring constants
b = np.array([5.0, 5.0, 5.0])  # damping coefficients

# Simulation loop
q_history = []
for step in range(num_steps):
    M = inertia_tensor(q)
    C = coriolis_matrix(q, q_dot)
    G = gravity_torques(q)
    J = jacobian_matrix(q)
    
    # Compute joint torques
    spring_torque = -k * q  # spring force
    damping_torque = -b * q_dot  # damping force
    tau = spring_torque + damping_torque + tau_act
    
    # Compute joint accelerations
    q_ddot = np.linalg.inv(M).dot(tau - C.dot(q_dot) - G + J.T.dot(f_ext))
    
    # Update joint states
    q_dot += q_ddot * dt
    q += q_dot * dt
    
    # Store history
    q_history.append(q.copy())

# Plot results
import matplotlib.pyplot as plt
q_history = np.array(q_history)

plt.figure()
for i in range(3):
    plt.plot(q_history[:, i], label=f'q{i+1}')
plt.legend()
plt.title('Joint Angles Over Time')
plt.xlabel('Time Steps')
plt.ylabel('Joint Angles (rad)')
plt.grid()
plt.show()
