import numpy as np
import matplotlib.pyplot as plt

import matplotlib.font_manager as fm

# 1. フォントを明示的に設定
font_path = 'C:/Windows/Fonts/meiryo.ttc'  # Linux の例
# Windows の場合: 'C:/Windows/Fonts/meiryo.ttc' など

font_prop = fm.FontProperties(fname=font_path)
# plt.rcParams['font.family'] = font_prop.get_name()


# --- モータ & 注入パラメータ ---------------------------------------
Ld, Lq     = 3e-3, 4e-3           # [H] d/q軸インダクタンス
L0, L1     = (Ld + Lq) / 2, (Ld - Lq) / 2 # 平均・差動インダクタンス
Vh, f_h    = 10.0, 1000.0         # [V] 注入電圧振幅, [Hz] 注入周波数
omega_h    = 2 * np.pi * f_h      # [rad/s] 注入角周波数

dt, Tsim   = 2e-5, 0.1            # [s] シミュレーション刻み幅, [s] シミュレーション時間 (追従確認のため少し延長)
steps      = int(Tsim / dt)       # シミュレーションステップ数

# --- PLL ゲイン設計（2次同定則）------------------------------------
# 設計指針：ζ ≈ 0.7, ω_n ≈ 2π * 200 Hz (PLL帯域の目安)
zeta_desired = 0.7
# 目標PLL自然周波数 [rad/s]
# コメントの「200 Hz」を自然周波数として採用
omega_n_desired = 2 * np.pi * 200

# Kp, Ki を設計値から計算
Kp = 2 * zeta_desired * omega_n_desired
Ki = omega_n_desired**2

print(f"計算された Kp: {Kp:.2f}")
print(f"計算された Ki: {Ki:.2e}")

# k=1 と仮定されていた感度ゲインを明示化
k_sensor = 0.001

# --- ノイズパラメータ ----------------------------------------------
# 電流測定ノイズの標準偏差 (例として設定)
noise_std_dev = 0.01

# --- 初期値 --------------------------------------------------------
# 真のロータ位置 (動的に変化させる)
# 例: 初期位置 0.3 rad, 一定速度 10 rad/s で回転
theta_true_initial = 0.3
omega_true_constant = 10.0 # [rad/s] 真のロータの角速度

theta_hat  = 0.0                  # [rad] 推定ロータ位置 初期値
integ      = 0.0                  # PIコントローラ積分項 初期値
phi_h      = 0.0                  # [rad] HF注入電圧の位相 初期値

# --- 履歴保存用リスト ------------------------------------------------
theta_true_hist = []
theta_hat_hist  = []
error_hist      = [] # 推定誤差 (theta_hat - theta_true)
i_delta_hist    = [] # HF電流応答
omega_hat_hist  = []
time_hist       = []

# --- シミュレーションループ ------------------------------------------
for i in range(steps):
    current_time = i * dt

    # 真のロータ位置を更新 (一定速度で回転)
    theta_true = theta_true_initial + omega_true_constant * current_time
    # 角度を [0, 2π) に正規化
    theta_true = theta_true % (2 * np.pi)

    # 1) HF 電圧印加 (現在のシミュレーションでは直接使われていないが、概念として)
    # v_gamma = Vh * np.cos(phi_h)
    # v_delta = Vh * np.sin(phi_h)

    # 2) HF 電流応答（簡易モデル：定常値のみ）
    # i_delta は推定ロータ位置と真のロータ位置の誤差に依存する成分
    i_delta_ideal = (L1 / (L0**2 + L1**2)) * Vh * np.sin(2 * (theta_hat - theta_true))

    # ノイズを追加
    i_delta = i_delta_ideal + np.random.normal(0, noise_std_dev)

    # 3) 誤差と PI-PLL
    # ここでの誤差は i_delta (k_sensor倍される)
    err = k_sensor * i_delta
    integ += err * dt
    omega_hat = Kp * err + Ki * integ
    theta_hat += omega_hat * dt

    # 4) 進行波位相を更新（実機では固定周波数発振器）
    phi_h = (phi_h + omega_h * dt) % (2 * np.pi)

    # 履歴を保存
    theta_true_hist.append(theta_true)
    theta_hat_hist.append(theta_hat)
    error_hist.append(theta_hat - theta_true) # 推定誤差を直接保存
    i_delta_hist.append(i_delta)
    omega_hat_hist.append(omega_hat)
    time_hist.append(current_time)

# --- 結果の可視化 --------------------------------------------------
plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(time_hist, theta_true_hist, label='True rotor position (theta_true)', linestyle='--')
plt.plot(time_hist, theta_hat_hist, label='Estimated rotor position (theta_hat)')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.title('Rotor Position Estimation')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(time_hist, error_hist, label='Estimation error (theta_hat - theta_true)')
plt.xlabel('Time [s]')
plt.ylabel('Error [rad]')
plt.title('PLL Estimation Error')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(time_hist, i_delta_hist, label='HF current response (i_delta)')
plt.xlabel('Time [s]')
plt.ylabel('Current [A]')
plt.title('PLL Error Signal (HF Current Response)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(time_hist, omega_hat_hist, label='Estimated angular velocity (omega_hat)')
plt.axhline(y=omega_true_constant, color='r', linestyle='--', label='True angular velocity (omega_true)')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.title('Estimated Angular Velocity')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Review of simulation results
print(f"\nFinal estimation error: {error_hist[-1]:.4e} rad")
print(f"Final estimated angular velocity: {omega_hat_hist[-1]:.2f} rad/s")
print(f"True angular velocity: {omega_true_constant:.2f} rad/s")
