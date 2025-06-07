import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import cholesky

# ───────────────────────────────────────────────
# 0) パラメータ設定
# ───────────────────────────────────────────────
Va   = 25.0          # HF 電圧振幅 [V]
f_h  = 1000.0        # HF 周波数 [Hz]
omega_h = 2*np.pi*f_h

Ld, Lq = 1e-3, 1.4e-3    # d/q インダクタンス [H]
R_s   = 0.03              # ステータ抵抗 [Ω]

Ts   = 1/6000*2              # サンプリング周期 [s]
Tend = 0.05              # 解析時間 [s]

# 真値
theta_true = 0.0         # True initial electrical angle
rpm = 2000 #[rpm]
Pn = 4 #Motor Pole
omega_true = rpm / 60 * (2*np.pi) * Pn # True electrical angular velocity
print(f"True electrical angular velocity: {omega_true:.2f} rad/s")

# 推定初期値を設定
theta_est_initial = np.deg2rad(30) # Initial estimated electrical angle
omega_est_initial = omega_true            # Initial estimated electrical angular velocity

# ───────────────────────────────────────────────
# 1) ユーティリティ関数
# ───────────────────────────────────────────────
def wrap(th):
    """Wrap angle to -pi to pi range"""
    return (th + np.pi) % (2*np.pi) - np.pi

def rot(alpha, vec):
    """Coordinate transformation (Rotation matrix)"""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([ c*vec[0] - s*vec[1],
                      s*vec[0] + c*vec[1] ])

# ───────────────────────────────────────────────
# 2) UKF クラス
# ───────────────────────────────────────────────
class UKF:
    def __init__(self, Ts, Ld, Lq, Rs, initial_x, initial_P, Q, R, alpha=1e-3, beta=2, kappa=0):
        self.Ts = Ts
        self.Ld = Ld
        self.Lq = Lq
        self.Rs = Rs

        self.x_hat = initial_x  # State estimate [theta, omega, Id, Iq]
        self.P = initial_P      # State error covariance matrix

        self.Q = Q              # Process noise covariance matrix
        self.R = R              # Measurement noise covariance matrix

        self.n = len(initial_x) # State dimension
        self.m = R.shape[0]     # Measurement dimension

        # UKF parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n

        # Weights for sigma points
        self.Wm = np.zeros(2 * self.n + 1)
        self.Wc = np.zeros(2 * self.n + 1)

        self.Wm[0] = self.lambda_ / (self.n + self.lambda_)
        self.Wc[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * self.n + 1):
            self.Wm[i] = 1 / (2 * (self.n + self.lambda_))
            self.Wc[i] = 1 / (2 * (self.n + self.lambda_))

    def _generate_sigma_points(self, x, P):
        """Generates sigma points from mean x and covariance P"""
        # Add a small epsilon to the diagonal to ensure positive definiteness for cholesky
        S = cholesky((self.n + self.lambda_) * (P + np.eye(self.n) * 1e-9))
        
        sigma_points = np.zeros((2 * self.n + 1, self.n))
        sigma_points[0] = x
        for i in range(self.n):
            sigma_points[i + 1] = x + S[:, i]
            sigma_points[self.n + i + 1] = x - S[:, i]
        return sigma_points

    def _state_transition_function(self, x, V_alpha, V_beta):
        """Non-linear state transition function f(x, u)"""
        theta, omega, Id, Iq = x

        # Vd, Vq are calculated from estimated angle and input voltages
        Vd_est = V_alpha * np.cos(theta) + V_beta * np.sin(theta)
        Vq_est = -V_alpha * np.sin(theta) + V_beta * np.cos(theta)

        d_theta = omega
        d_omega = 0.0 # Assume constant speed (or very slow change)
        d_Id = (1/self.Ld) * (Vd_est - self.Rs*Id + omega*self.Lq*Iq)
        d_Iq = (1/self.Lq) * (Vq_est - self.Rs*Iq - omega*self.Ld*Id)

        # Discrete-time update
        x_next = np.array([
            theta + d_theta * self.Ts,
            omega + d_omega * self.Ts,
            Id + d_Id * self.Ts,
            Iq + d_Iq * self.Ts
        ])
        x_next[0] = wrap(x_next[0]) # Wrap angle
        return x_next

    def _measurement_function(self, x):
        """Non-linear measurement function h(x)"""
        theta, _, Id, Iq = x
        return np.array([
            Id * np.cos(theta) - Iq * np.sin(theta), # I_alpha
            Id * np.sin(theta) + Iq * np.cos(theta)  # I_beta
        ])

    def predict(self, V_alpha, V_beta):
        """Prediction step of UKF"""
        # Generate sigma points
        sigma_points = self._generate_sigma_points(self.x_hat, self.P)

        # Propagate sigma points through the state transition function
        propagated_sigma_points = np.array([
            self._state_transition_function(sp, V_alpha, V_beta) for sp in sigma_points
        ])

        # Calculate predicted mean
        self.x_hat_minus = np.sum(self.Wm[:, np.newaxis] * propagated_sigma_points, axis=0)
        self.x_hat_minus[0] = wrap(self.x_hat_minus[0]) # Ensure wrapped mean angle

        # Calculate predicted covariance
        P_minus = np.zeros((self.n, self.n))
        for i in range(2 * self.n + 1):
            diff = propagated_sigma_points[i] - self.x_hat_minus
            P_minus += self.Wc[i] * np.outer(diff, diff)
        self.P_minus = P_minus + self.Q

    def update(self, measurements):
        """Update step of UKF"""
        # Generate sigma points from predicted mean and covariance
        sigma_points_minus = self._generate_sigma_points(self.x_hat_minus, self.P_minus)

        # Propagate sigma points through the measurement function
        predicted_measurements = np.array([
            self._measurement_function(sp) for sp in sigma_points_minus
        ])

        # Calculate predicted measurement mean
        z_hat = np.sum(self.Wm[:, np.newaxis] * predicted_measurements, axis=0)

        # Calculate innovation covariance S
        S = np.zeros((self.m, self.m))
        for i in range(2 * self.n + 1):
            diff = predicted_measurements[i] - z_hat
            S += self.Wc[i] * np.outer(diff, diff)
        S += self.R

        # Calculate cross-covariance P_xz
        P_xz = np.zeros((self.n, self.m))
        for i in range(2 * self.n + 1):
            diff_x = sigma_points_minus[i] - self.x_hat_minus
            diff_z = predicted_measurements[i] - z_hat
            P_xz += self.Wc[i] * np.outer(diff_x, diff_z)

        # Calculate Kalman Gain
        K = P_xz @ np.linalg.inv(S)

        # Update state estimate
        y = measurements - z_hat # Measurement residual
        self.x_hat = self.x_hat_minus + K @ y
        self.x_hat[0] = wrap(self.x_hat[0]) # Ensure updated angle is wrapped

        # Update covariance matrix
        self.P = self.P_minus - K @ S @ K.T

# ───────────────────────────────────────────────
# 3) 状態・ログ
# ───────────────────────────────────────────────
# True dq currents (for simulation)
Id_true, Iq_true = 0.0, 0.0

t_log, th_est_log, th_true_log, err_deg_log = [], [], [], []
omega_est_log = []

# UKF Initialization
# State: [theta, omega, Id, Iq]
initial_x_ukf = np.array([theta_est_initial, omega_est_initial, Id_true, Iq_true])
# Initial covariance matrix (express uncertainty with large values)
initial_P_ukf = np.diag([np.deg2rad(90)**2, (100.0)**2, 1.0**2, 1.0**2]) # Initial uncertainty for theta, omega, Id, Iq
# Process noise covariance matrix (uncertainty of state transition model)
Q_ukf = np.diag([
    (np.deg2rad(0.1))**2, # Noise for theta (very small)
    (10.0)**2,            # Noise for omega (uncertainty in speed variation)
    (0.1)**2,             # Noise for Id
    (0.1)**2              # Noise for Iq
])
# Measurement noise covariance matrix (uncertainty of current measurement)
R_ukf = np.diag([
    (0.01)**2, # Noise for I_alpha
    (0.01)**2  # Noise for I_beta
])

ukf = UKF(Ts, Ld, Lq, R_s, initial_x_ukf, initial_P_ukf, Q_ukf, R_ukf)


# ───────────────────────────────────────────────
# 4) ループ
# ───────────────────────────────────────────────
steps = int(Tend / Ts)
for k in range(steps):
    t = k * Ts

    # === (1) HF voltage generation (applied based on estimated angle) ===
    # Here, use theta_est from UKF
    Vgamma = Va * np.sin(omega_h * t)
    Vdelta = 0.0
    
    # Use the current estimated angle from UKF
    V_alpha, V_beta = rot(ukf.x_hat[0], np.array([Vgamma, Vdelta]))

    # === (2) dq current model (True motor model) ===
    # Calculate currents with true angle and speed
    V_d_true, V_q_true = rot(-theta_true, np.array([V_alpha, V_beta]))
    
    dId_true = (1/Ld)*(V_d_true - R_s*Id_true + omega_true*Lq*Iq_true)
    dIq_true = (1/Lq)*(V_q_true - R_s*Iq_true - omega_true*Ld*Id_true)
    Id_true += dId_true * Ts
    Iq_true += dIq_true * Ts
    
    # True alpha-beta currents (used as measurements for UKF)
    I_alpha_meas, I_beta_meas = rot(theta_true, np.array([Id_true, Iq_true]))

    # === (3) UKF processing ===
    ukf.predict(V_alpha, V_beta)
    ukf.update(np.array([I_alpha_meas, I_beta_meas]))

    # Get estimated angle and speed
    theta_est = ukf.x_hat[0]
    omega_est = ukf.x_hat[1]

    # === (0) Update true electrical angle ===
    theta_true += omega_true * Ts
    theta_true = wrap(theta_true) # Wrap angle to -pi to pi range

    # === Log data ===
    t_log.append(t)
    th_est_log.append(np.rad2deg(theta_est))
    th_true_log.append(np.rad2deg(theta_true))
    
    th_diff = theta_est - theta_true
    th_diff = wrap(th_diff)
    err_deg_log.append(np.rad2deg(th_diff))
    omega_est_log.append(omega_est)


# ───────────────────────────────────────────────
# 5) 描画
# ───────────────────────────────────────────────
plt.rcParams['font.family'] = 'DejaVu Sans' # Set font for non-TeX plotting
plt.rcParams['font.size'] = 12

plt.figure(figsize=(10, 6))
plt.plot(t_log, th_est_log, label=r'$\hat\theta_{est}$ (Estimated)')
plt.plot(t_log, th_true_log, label=r'$\theta_{true}$ (True)')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'Electrical angle [deg]')
plt.title(r'Electrical Angle Tracking with UKF')
plt.grid(); plt.legend()

plt.figure(figsize=(10, 6))
plt.plot(t_log, err_deg_log)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Phase\;error\;[deg]$')
plt.title(r'Phase Error Convergence with UKF')
plt.grid()

plt.figure(figsize=(10, 6))
plt.plot(t_log, omega_est_log, label=r'$\hat\omega_{est}$ (Estimated)')
plt.axhline(y=omega_true, color='r', linestyle='--', label=r'$\omega_{true}$ (True)')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'Electrical Angular Velocity [rad/s]')
plt.title(r'Electrical Angular Velocity Tracking with UKF')
plt.grid(); plt.legend()

plt.tight_layout()
plt.show()
