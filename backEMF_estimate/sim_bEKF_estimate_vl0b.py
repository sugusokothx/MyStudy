import numpy as np
import matplotlib.pyplot as plt

# --- EKF Class based on the provided literature (No changes) ---
class ExtendedKalmanFilter:
    """
    An implementation of the Extended Kalman Filter (EKF) as described in Section 3 of the literature.
    """
    def __init__(self, x, P, Q, R):
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R

    def predict(self, u, F_k, f_func):
        # State prediction based on Eq. (4)
        self.x = f_func(self.x, u)
        # Error covariance prediction based on Eq. (5)
        self.P = F_k @ self.P @ F_k.T + self.Q

    def update(self, z, H_k, h_func):
        # Kalman gain calculation based on Eq. (7)
        S = H_k @ self.P @ H_k.T + self.R
        K = self.P @ H_k.T @ np.linalg.inv(S)
        # State vector update based on Eq. (10)
        y = z - h_func(self.x)
        self.x = self.x + K @ y
        # Error covariance update based on Eq. (9)
        self.P = self.P - K @ H_k @ self.P

# --- Model functions for the simulation (No changes) ---
def state_transition_function(x, u):
    dt = 0.1
    yaw = x[2]
    vel = u[0]
    return np.array([
        x[0] + dt * vel * np.cos(yaw),
        x[1] + dt * vel * np.sin(yaw),
        x[2] + dt * u[1]
    ])

def measurement_function(x):
    return np.array([x[0], x[1]])

def jacobian_F(x, u):
    dt = 0.1
    yaw = x[2]
    vel = u[0]
    F = np.eye(3)
    F[0, 2] = -dt * vel * np.sin(yaw)
    F[1, 2] = dt * vel * np.cos(yaw)
    return F

def jacobian_H():
    return np.array([[1, 0, 0], [0, 1, 0]])

# --- Simulation Execution ---
if __name__ == '__main__':
    # Simulation settings
    SIM_STEPS = 150
    dt = 0.1

    # Initial state
    true_x = np.array([0.0, 0.0, 0.0]) # Ground truth [x, y, yaw]
    estimated_x = true_x.copy() # Initial state for EKF

    # EKF parameters
    P = np.eye(3) # Initial error covariance
    Q = np.diag([0.1, 0.1, np.deg2rad(1.0)])**2 # Process noise
    R = np.diag([0.5, 0.5])**2 # Measurement noise

    # Control input [velocity, angular velocity]
    u = np.array([1.0, 0.1])

    # Create an EKF instance
    ekf = ExtendedKalmanFilter(x=estimated_x, P=P, Q=Q, R=R)

    # Lists to store history for plotting
    history_true_x = [true_x.copy()]
    history_estimated_x = [estimated_x.copy()]
    history_measurements = []

    print("Starting simulation...")

    # Main loop
    for i in range(SIM_STEPS):
        # 1. Update ground truth
        true_x = state_transition_function(true_x, u)

        # 2. Generate a noisy measurement
        z = measurement_function(true_x) + np.random.multivariate_normal(np.zeros(2), R)

        # 3. EKF Predict step
        F_k = jacobian_F(ekf.x, u)
        ekf.predict(u=u, F_k=F_k, f_func=state_transition_function)

        # 4. EKF Update step
        ekf.update(z=z, H_k=jacobian_H(), h_func=measurement_function)

        # 5. Store results
        history_true_x.append(true_x.copy())
        history_estimated_x.append(ekf.x.copy())
        history_measurements.append(z)

    print("Simulation finished. Plotting results.")

    # Convert lists to NumPy arrays for easier slicing
    history_true_x = np.array(history_true_x)
    history_estimated_x = np.array(history_estimated_x)
    history_measurements = np.array(history_measurements)

    # --- Plotting the results ---
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.figure(figsize=(10, 8))

    # Measurements
    plt.scatter(history_measurements[:, 0], history_measurements[:, 1],
                s=20, c='gray', alpha=0.7, label='Measurements')

    # Ground Truth
    plt.plot(history_true_x[:, 0], history_true_x[:, 1],
             'g-', linewidth=2, label='Ground Truth')

    # EKF Estimate
    plt.plot(history_estimated_x[:, 0], history_estimated_x[:, 1],
             'b--', linewidth=2, label='EKF Estimate')

    plt.title('Extended Kalman Filter (EKF) Simulation Result')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()