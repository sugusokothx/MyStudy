import numpy as np
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    """Minimal EKF based on the user‑supplied interface."""
    def __init__(self, x, P, Q, R, f_func, h_func):
        self.x = x.astype(float)
        self.P = P.astype(float)
        self.Q = Q.astype(float)
        self.R = R.astype(float)
        self.f_func = f_func
        self.h_func = h_func
        self.n = x.size

    def predict(self, u, F_k):
        # state prediction
        self.x = self.f_func(self.x, u)
        # covariance prediction
        self.P = F_k @ self.P @ F_k.T + self.Q

    def update(self, z, H_k):
        # Kalman gain
        S = H_k @ self.P @ H_k.T + self.R
        K = self.P @ H_k.T @ np.linalg.inv(S)
        # state update
        y = z - self.h_func(self.x)
        self.x = self.x + K @ y
        # covariance update
        self.P = self.P - K @ H_k @ self.P


# ------------------------------------------------------------------
# model‑specific functions (2‑D position + heading)
# ------------------------------------------------------------------
def state_transition_function(x, u):
    """x = [pos_x, pos_y, yaw], u = [velocity, yaw_rate]"""
    dt = 0.1
    vel, yaw_rate = u
    yaw = x[2]
    return np.array([
        x[0] + dt * vel * np.cos(yaw),
        x[1] + dt * vel * np.sin(yaw),
        x[2] + dt * yaw_rate
    ])

def measurement_function(x):
    """observe (x, y) only"""
    return x[:2]

def jacobian_F(x, u):
    dt = 0.1
    vel = u[0]
    yaw = x[2]
    F = np.eye(3)
    F[0, 2] = -dt * vel * np.sin(yaw)
    F[1, 2] =  dt * vel * np.cos(yaw)
    return F

H_const = np.array([[1., 0., 0.],
                    [0., 1., 0.]])

# ------------------------------------------------------------------
# simulation
# ------------------------------------------------------------------
def run_sim():
    # initial conditions
    x0 = np.array([0., 0., 0.])
    P0 = np.eye(3)
    Q = (np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2)
    R = (np.diag([0.5, 0.5]) ** 2)

    ekf = ExtendedKalmanFilter(x0, P0, Q, R,
                               state_transition_function,
                               measurement_function)

    true_x = x0.copy()
    u = np.array([1.0, 0.1])  # velocity [m/s], yaw_rate [rad/s]

    true_hist, est_hist = [], []
    steps = 100
    for _ in range(steps):
        # ground‑truth propagation
        true_x = state_transition_function(true_x, u)

        # noisy observation
        z = measurement_function(true_x) + np.random.multivariate_normal(np.zeros(2), R)

        # EKF prediction + update
        Fk = jacobian_F(ekf.x, u)
        ekf.predict(u, Fk)
        ekf.update(z, H_const)

        true_hist.append(true_x.copy())
        est_hist.append(ekf.x.copy())

    true_hist = np.asarray(true_hist)
    est_hist = np.asarray(est_hist)

    # plot trajectory
    plt.figure()
    plt.plot(true_hist[:, 0], true_hist[:, 1], label="True trajectory")
    plt.plot(est_hist[:, 0], est_hist[:, 1], label="EKF estimate")
    plt.xlabel("X position [m]")
    plt.ylabel("Y position [m]")
    plt.title("EKF position tracking demo")
    plt.legend()
    plt.grid(True)
    plt.show()

run_sim()
