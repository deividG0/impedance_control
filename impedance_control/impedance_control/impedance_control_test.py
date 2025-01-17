import numpy as np
import matplotlib.pyplot as plt

class ImpedanceController:
    def __init__(self, M, D, K, dt):
        """
        Initialize the impedance controller.
        
        Args:
            M: Desired mass matrix (task space)
            D: Desired damping matrix (task space)
            K: Desired stiffness matrix (task space)
            dt: Time step for simulation
        """
        self.M = np.array(M)
        self.D = np.array(D)
        self.K = np.array(K)
        self.dt = dt
        self.x_d = np.zeros((3, 1))  # Desired position
        self.v_d = np.zeros((3, 1))  # Desired velocity
        self.a_d = np.zeros((3, 1))  # Desired acceleration

    def compute_control(self, x, v, f_ext):
        """
        Compute the control force for the manipulator.

        Args:
            x: Current position (3x1 vector)
            v: Current velocity (3x1 vector)
            f_ext: External force applied (3x1 vector)

        Returns:
            F_ctrl: Control force to apply (3x1 vector)
        """
        # Compute error terms
        x_error = x - self.x_d
        v_error = v - self.v_d
        
        # Compute desired force
        F_desired = self.M @ self.a_d - self.D @ v_error - self.K @ x_error

        # Include external forces
        F_ctrl = F_desired + f_ext
        return F_ctrl

# Example usage
if __name__ == "__main__":
    # Desired impedance parameters
    M = np.diag([1.0, 1.0, 1.0])  # Desired mass
    D = np.diag([10.0, 10.0, 10.0])  # Desired damping
    K = np.diag([100.0, 100.0, 100.0])  # Desired stiffness
    dt = 0.01  # Time step
    
    # Initialize controller
    controller = ImpedanceController(M, D, K, dt)

    # Simulation parameters
    x = np.array([[0.0], [0.0], [0.0]])  # Initial position
    v = np.array([[0.0], [0.0], [0.0]])  # Initial velocity
    f_ext = np.array([[10.0], [150.0], [0.0]])  # External force

    # Desired trajectory
    controller.x_d = np.array([[1.0], [1.0], [1.0]])  # Target position

    # Data storage for plotting
    time_steps = []
    positions = []
    velocities = []

    # Simulate
    for t in np.arange(0, 10, dt):
        F_ctrl = controller.compute_control(x, v, f_ext)
        
        # Update state using simple Euler integration
        a = np.linalg.inv(M) @ F_ctrl  # Acceleration
        v += a * dt
        x += v * dt

        # Store data
        time_steps.append(t)
        positions.append(x.flatten())
        velocities.append(v.flatten())

    # Convert lists to numpy arrays for easier manipulation
    positions = np.array(positions)
    velocities = np.array(velocities)

    # Plot position and velocity
    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    # Plot positions
    axes[0].plot(time_steps, positions[:, 0], label="x (Position)")
    axes[0].plot(time_steps, positions[:, 1], label="y (Position)")
    axes[0].plot(time_steps, positions[:, 2], label="z (Position)")
    axes[0].set_title("Position vs Time")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Position (m)")
    axes[0].legend()
    axes[0].grid()

    # Plot velocities
    axes[1].plot(time_steps, velocities[:, 0], label="x (Velocity)")
    axes[1].plot(time_steps, velocities[:, 1], label="y (Velocity)")
    axes[1].plot(time_steps, velocities[:, 2], label="z (Velocity)")
    axes[1].set_title("Velocity vs Time")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].legend()
    axes[1].grid()

    # Show plots
    plt.tight_layout()
    plt.show()
