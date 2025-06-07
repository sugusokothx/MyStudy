import numpy as np
import matplotlib.pyplot as plt

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
omega_est_initial = 0.0            # Initial estimated electrical angular velocity

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
# 2) EKF クラス
# ───────────────────────────────────────────────
class EKF:
    def __init__(self, Ts, Ld, Lq, Rs, initial_x, initial_P, Q, R):
        self.Ts = Ts
        self.Ld = Ld
        self.Lq = Lq
        self.Rs = Rs

        self.x_hat = initial_x  # State estimate [theta, omega, Id, Iq]
        self.P = initial_P      # State error covariance matrix

        self.Q = Q              # Process noise covariance matrix
        self.R = R              # Measurement noise covariance matrix

    def predict(self, V_alpha, V_beta):
        """
        Prediction step
        x_hat_k+1 = f(x_hat_k, u_k)
        P_k+1 = F_k * P_k * F_k^T + Q
        """
        theta, omega, Id, Iq = self.x_hat

        # State equations f(x, u)
        # Vd, Vq are calculated from estimated angle and input voltages
        Vd_est = V_alpha * np.cos(theta) + V_beta * np.sin(theta)
        Vq_est = -V_alpha * np.sin(theta) + V_beta * np.cos(theta)

        d_theta = omega
        d_omega = 0.0 # Assume constant speed (or very slow change)
        d_Id = (1/self.Ld) * (Vd_est - self.Rs*Id + omega*self.Lq*Iq)
        d_Iq = (1/self.Lq) * (Vq_est - self.Rs*Iq - omega*self.Ld*Id)

        # Update state estimate (Euler method)
        self.x_hat[0] += d_theta * self.Ts
        self.x_hat[0] = wrap(self.x_hat[0]) # Wrap angle
        self.x_hat[1] += d_omega * self.Ts
        self.x_hat[2] += d_Id * self.Ts
        self.x_hat[3] += d_Iq * self.Ts

        # Jacobian F (linearization of state transition matrix)
        # F = I + Ts * F_continuous
        # F_continuous = df/dx
        # Vd_est, Vq_est depend on theta, so their partial derivatives are also considered
        dVd_dtheta = -V_alpha * np.sin(theta) + V_beta * np.cos(theta) # = Vq_est
        dVq_dtheta = -V_alpha * np.cos(theta) - V_beta * np.sin(theta) # = -Vd_est

        F_continuous = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [(1/self.Ld) * dVd_dtheta, (self.Lq*Iq)/self.Ld, -self.Rs/self.Ld, (omega*self.Lq)/self.Ld],
            [(1/self.Lq) * dVq_dtheta, (-self.Ld*Id)/self.Lq, (-omega*self.Ld)/self.Lq, -self.Rs/self.Lq]
        ])
        
        F_k = np.eye(4) + F_continuous * self.Ts

        # Update covariance matrix
        self.P = F_k @ self.P @ F_k.T + self.Q

    def update(self, measurements):
        """
        Update step
        K_k = P_k * H_k^T * (H_k * P_k * H_k^T + R)^-1
        x_hat_k = x_hat_k + K_k * (z_k - h(x_hat_k))
        P_k = (I - K_k * H_k) * P_k
        """
        theta, omega, Id, Iq = self.x_hat
        I_alpha_meas, I_beta_meas = measurements

        # Measurement equations h(x)
        h_x = np.array([
            Id * np.cos(theta) - Iq * np.sin(theta), # I_alpha
            Id * np.sin(theta) + Iq * np.cos(theta)  # I_beta
        ])

        # Jacobian H (linearization of measurement matrix)
        # H = dh/dx
        H_k = np.array([
            [-Id * np.sin(theta) - Iq * np.cos(theta), 0, np.cos(theta), -np.sin(theta)], # d(I_alpha)/dx
            [ Id * np.cos(theta) - Iq * np.sin(theta), 0, np.sin(theta),  np.cos(theta)]  # d(I_beta)/dx
        ])
        
        # Calculate Kalman Gain
        S = H_k @ self.P @ H_k.T + self.R
        K = self.P @ H_k.T @ np.linalg.inv(S)

        # Update state estimate
        y = measurements - h_x # Measurement residual
        self.x_hat = self.x_hat + K @ y

        # Update covariance matrix
        self.P = (np.eye(4) - K @ H_k) @ self.P

# ───────────────────────────────────────────────
# 3) 状態・ログ
# ───────────────────────────────────────────────
# True dq currents (for simulation)
Id_true, Iq_true = 0.0, 0.0

t_log, th_est_log, th_true_log, err_deg_log = [], [], [], []
omega_est_log = []

# EKF Initialization
# State: [theta, omega, Id, Iq]
initial_x_ekf = np.array([theta_est_initial, omega_est_initial, Id_true, Iq_true])
# Initial covariance matrix (express uncertainty with large values)
initial_P_ekf = np.diag([np.deg2rad(90)**2, (100.0)**2, 1.0**2, 1.0**2]) # Initial uncertainty for theta, omega, Id, Iq
# Process noise covariance matrix (uncertainty of state transition model)
Q_ekf = np.diag([
    (np.deg2rad(0.1))**2, # Noise for theta (very small)
    (10.0)**2,            # Noise for omega (uncertainty in speed variation)
    (0.1)**2,             # Noise for Id
    (0.1)**2              # Noise for Iq
])
# Measurement noise covariance matrix (uncertainty of current measurement)
R_ekf = np.diag([
    (0.01)**2, # Noise for I_alpha
    (0.01)**2  # Noise for I_beta
])

ekf = EKF(Ts, Ld, Lq, R_s, initial_x_ekf, initial_P_ekf, Q_ekf, R_ekf)


# ───────────────────────────────────────────────
# 4) ループ
# ───────────────────────────────────────────────
steps = int(Tend / Ts)
for k in range(steps):
    t = k * Ts

    # === (1) HF voltage generation (applied based on estimated angle) ===
    # Here, use theta_est from EKF
    Vgamma = Va * np.sin(omega_h * t)
    Vdelta = 0.0
    
    # Use the current estimated angle from EKF
    V_alpha, V_beta = rot(ekf.x_hat[0], np.array([Vgamma, Vdelta]))

    # === (2) dq current model (True motor model) ===
    # Calculate currents with true angle and speed
    V_d_true, V_q_true = rot(-theta_true, np.array([V_alpha, V_beta]))
    
    dId_true = (1/Ld)*(V_d_true - R_s*Id_true + omega_true*Lq*Iq_true)
    dIq_true = (1/Lq)*(V_q_true - R_s*Iq_true - omega_true*Ld*Id_true)
    Id_true += dId_true * Ts
    Iq_true += dIq_true * Ts
    
    # True alpha-beta currents (used as measurements for EKF)
    I_alpha_meas, I_beta_meas = rot(theta_true, np.array([Id_true, Iq_true]))

    # === (3) EKF processing ===
    ekf.predict(V_alpha, V_beta)
    ekf.update(np.array([I_alpha_meas, I_beta_meas]))

    # Get estimated angle and speed
    theta_est = ekf.x_hat[0]
    omega_est = ekf.x_hat[1]

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
plt.title(r'Electrical Angle Tracking with EKF')
plt.grid(); plt.legend()

plt.figure(figsize=(10, 6))
plt.plot(t_log, err_deg_log)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Phase\;error\;[deg]$')
plt.title(r'Phase Error Convergence with EKF')
plt.grid()

plt.figure(figsize=(10, 6))
plt.plot(t_log, omega_est_log, label=r'$\hat\omega_{est}$ (Estimated)')
plt.axhline(y=omega_true, color='r', linestyle='--', label=r'$\omega_{true}$ (True)')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'Electrical Angular Velocity [rad/s]')
plt.title(r'Electrical Angular Velocity Tracking with EKF')
plt.grid(); plt.legend()

plt.tight_layout()
plt.show()

