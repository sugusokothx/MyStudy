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

# PLL ゲイン
Kp = 600.0*1
Ki = 8e5*1                 # Ki ≠ 0 で定常誤差を除去

tau_lpf = 1e-4*2           # 復調 LPF 時定数 [s]
alpha_lpf = Ts / (tau_lpf + Ts)

# 真値
theta_true = 0.0         # 定常角
rpm = 2000 #[rpm]
Pn = 4 #Motor Pole
omega_true = rpm / 60 * (2*np.pi) * Pn
# omega_true = 100.0         # 速度 0  (≠0 にしても可)
print(omega_true)

# 推定初期値を -70° に
theta_est = np.deg2rad(30)
omega_est = 0.0

# ───────────────────────────────────────────────
# 1) ユーティリティ
# ───────────────────────────────────────────────
def wrap(th):
    return (th + np.pi) % (2*np.pi) - np.pi

def rot(alpha, vec):
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([ c*vec[0] - s*vec[1],
                      s*vec[0] + c*vec[1] ])

# ───────────────────────────────────────────────
# 2) 状態・ログ
# ───────────────────────────────────────────────
Id, Iq = 0.0, 0.0           # dq 電流
# IQ 復調用 LPF 状態 (sin と cos の 2成分ずつ)
ih_s_alpha = ih_s_beta = 0.0
ih_c_alpha = ih_c_beta = 0.0

int_err = 0.0               # PI 積分

t_log, th_log, err_deg_log , th_true_log= [], [], [], []

# ───────────────────────────────────────────────
# 3) ループ
# ───────────────────────────────────────────────
steps = int(Tend / Ts)
for k in range(steps):
    t = k * Ts

    # === (1) HF 電圧生成 ===
    Vgamma = Va * np.sin(omega_h * t)
    Vdelta = 0.0
    V_alpha, V_beta = rot(theta_est, np.array([Vgamma, Vdelta]))

    # === (2) dq 電流モデル (Euler, Rs & クロスターム有り) ===
    V_d, V_q = rot(-theta_true, np.array([V_alpha, V_beta]))
    dId = (1/Ld)*(V_d - R_s*Id + omega_true*Lq*Iq)
    dIq = (1/Lq)*(V_q - R_s*Iq - omega_true*Ld*Id)
    Id += dId * Ts
    Iq += dIq * Ts
    I_alpha, I_beta = rot(theta_true, np.array([Id, Iq]))

    # === (3) I/Q 復調 ===
    sin_ref = np.sin(omega_h * t)
    cos_ref = np.cos(omega_h * t)

    # --- I/Q demod (complex) ---
    ih_s_alpha += alpha_lpf * (I_alpha *  sin_ref - ih_s_alpha)  # Iチャネル
    ih_c_alpha += alpha_lpf * (I_alpha *  cos_ref - ih_c_alpha)  # Qチャネル
    ih_s_beta  += alpha_lpf * (I_beta  *  sin_ref - ih_s_beta)
    ih_c_beta  += alpha_lpf * (I_beta  *  cos_ref - ih_c_beta)

    # ×2 して実数部＝I, 虚数部＝Q の複素電流
    Ih_alpha = 2*(ih_s_alpha + 1j*ih_c_alpha)
    Ih_beta  = 2*(ih_s_beta  + 1j*ih_c_beta)

    # 固定 → γδ （複素ごと回転させても OK）
    Ih_gamma, Ih_delta = rot(-theta_est, np.array([Ih_alpha, Ih_beta]).real)  # 実数部でOK


    # # In‑phase (sin) branch
    # ih_s_alpha += alpha_lpf * (I_alpha * sin_ref - ih_s_alpha)
    # ih_s_beta  += alpha_lpf * (I_beta  * sin_ref - ih_s_beta)
    # # Quadrature (cos) branch
    # ih_c_alpha += alpha_lpf * (I_alpha * cos_ref - ih_c_alpha)
    # ih_c_beta  += alpha_lpf * (I_beta  * cos_ref - ih_c_beta)

    # # sin², cos² の平均 0.5 を補正 → ×2
    # Ih_alpha = 2 * ih_s_alpha   # ここでは sin 分のみ使用
    # Ih_beta  = 2 * ih_s_beta

    # # αβ → γδ
    # Ih_gamma, Ih_delta = rot(-theta_est, np.array([Ih_alpha, Ih_beta]))

    # === (4) PLL (PI) ===
    err = Ih_delta
    int_err += err * Ts # 積分器の入力は誤差そのもの
    omega_est = Kp * err + Ki * int_err # PLLの出力が推定角速度

    # === (5) 位相推定 ===
    theta_est += omega_est * Ts # 推定角速度でtheta_estを更新
    theta_est = wrap(theta_est)


    # === (0) 真の電気角更新 ===
    theta_true += omega_true * Ts
    theta_true = wrap(theta_true) # 角度を -pi から pi の範囲に収める

    # === ログ ===
    t_log.append(t)
 
    theta_est_deg = np.rad2deg(theta_est)
    th_log.append(theta_est_deg)

    th_true_log.append(np.rad2deg(theta_true))
    th_diff = theta_est - theta_true
    th_diff = wrap(th_diff)
    err_deg_log.append(np.rad2deg(th_diff))

# ───────────────────────────────────────────────
# 4) 描画
# ───────────────────────────────────────────────
# TeXを使用する設定
# plt.rcParams['text.usetex'] = True

plt.figure(figsize=(9,4))
plt.plot(t_log, th_log, label=r'$\hat\theta_{est}$')
plt.plot(t_log, th_true_log, label=r'$\theta_{true}$')


plt.xlabel(r'$Time [s]$')  # r'...' はraw文字列で、バックスラッシュのエスケープを防ぐ
plt.ylabel(r'Electrical angle [$^\circ$]') # 角度記号をTeXで記述
plt.title(r'Phase tracking with IQ demod + R$_s$ & cross terms') # 下付き文字や'&'のエスケープ
# plt.ylabel('Electrical angle [deg]')
# plt.xlabel('Time [s]')
# plt.title('Phase tracking with IQ demod + Rs & cross terms')
plt.grid(); plt.legend()

plt.figure(figsize=(9,4))
plt.plot(t_log, err_deg_log)
plt.ylabel(r'$Phase\;error\;[deg]$')
plt.xlabel(r'$Time [s]$')  # r'...' はraw文字列で、バックスラッシュのエスケープを防ぐ
plt.title(r'Phase error convergence')
plt.grid()

plt.tight_layout()
plt.show()
