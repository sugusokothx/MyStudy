import numpy as np
import matplotlib.pyplot as plt

# ───────────────────────────────────────────────
# ★ フィルタクラス定義 (上記で作成したものをここにコピー)
# ───────────────────────────────────────────────
class LPF:
    """1次ローパスフィルタ"""
    def __init__(self, alpha):
        self.alpha = alpha
        self.y = 0.0
    def filter(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y

class DFactorFilter:
    """D因子フィルタ（2次FIRノッチフィルタ）"""
    def __init__(self, omega_notch, Ts):
        self.a1 = -2 * np.cos(omega_notch * Ts)
        self.K = 1.0 / (2.0 + self.a1)
        self.x1 = 0.0
        self.x2 = 0.0
    def filter(self, x):
        y = self.K * (x + self.a1 * self.x1 + self.x2)
        self.x2, self.x1 = self.x1, x
        return y

class AdaptiveNotchFilter:
    """適応ノッチフィルタ (ANF)"""
    def __init__(self, initial_omega_notch, Ts, mu=1e-5, rho=0.98):
        self.mu = mu
        self.rho = rho
        self.a1 = -2 * np.cos(initial_omega_notch * Ts)
        self.x1, self.x2 = 0.0, 0.0
        self.y1, self.y2 = 0.0, 0.0
        self.s1, self.s2 = 0.0, 0.0
    def filter(self, x):
        y = (x + self.a1 * self.x1 + self.x2 - 
             self.rho * self.a1 * self.y1 - self.rho**2 * self.y2)
        s = self.x1 - self.rho * self.a1 * self.s1 - self.rho**2 * self.s2
        self.a1 = np.clip(self.a1 - self.mu * y * s, -1.99, 1.99)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        self.s2, self.s1 = self.s1, s
        return y

# ───────────────────────────────────────────────
# 0) パラメータ設定
# ───────────────────────────────────────────────
# ★★★ 使用するフィルタを選択 ★★★
FILTER_TYPE = 'DFactor'  # 'LPF', 'DFactor', または 'Adaptive'

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

# LPF用パラメータ (FILTER_TYPE = 'LPF' の場合のみ使用)
tau_lpf = 1e-4*2           # 復調 LPF 時定数 [s]
alpha_lpf = Ts / (tau_lpf + Ts)

# 真値
theta_true = 0.0         # 定常角
rpm = 2000 #[rpm]
Pn = 4 #Motor Pole
omega_true = rpm / 60 * (2*np.pi) * Pn
print(f"Motor speed: {omega_true:.2f} rad/s")

# 推定初期値を 30° に
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

# ★★★ 選択されたフィルタのインスタンスを4つ生成 ★★★
omega_notch = 2 * omega_h # 除去したいリップル周波数 (2ωh)

if FILTER_TYPE == 'LPF':
    filter_s_alpha = LPF(alpha_lpf)
    filter_c_alpha = LPF(alpha_lpf)
    filter_s_beta  = LPF(alpha_lpf)
    filter_c_beta  = LPF(alpha_lpf)
    title_suffix = 'LPF'
elif FILTER_TYPE == 'DFactor':
    filter_s_alpha = DFactorFilter(omega_notch, Ts)
    filter_c_alpha = DFactorFilter(omega_notch, Ts)
    filter_s_beta  = DFactorFilter(omega_notch, Ts)
    filter_c_beta  = DFactorFilter(omega_notch, Ts)
    title_suffix = 'D-Factor Filter'
elif FILTER_TYPE == 'Adaptive':
    # 適応フィルタのパラメータ (muは試行錯誤で調整が必要)
    mu_anf = 5e-5 
    rho_anf = 0.99
    filter_s_alpha = AdaptiveNotchFilter(omega_notch, Ts, mu=mu_anf, rho=rho_anf)
    filter_c_alpha = AdaptiveNotchFilter(omega_notch, Ts, mu=mu_anf, rho=rho_anf)
    filter_s_beta  = AdaptiveNotchFilter(omega_notch, Ts, mu=mu_anf, rho=rho_anf)
    filter_c_beta  = AdaptiveNotchFilter(omega_notch, Ts, mu=mu_anf, rho=rho_anf)
    title_suffix = 'Adaptive Notch Filter'

int_err = 0.0               # PI 積分
t_log, th_log, err_deg_log, th_true_log = [], [], [], []

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

    # --- 同期検波 ---
    demod_s_alpha = I_alpha * sin_ref
    demod_c_alpha = I_alpha * cos_ref
    demod_s_beta  = I_beta  * sin_ref
    demod_c_beta  = I_beta  * cos_ref

    # --- ★ フィルタリング (選択されたフィルタを使用) ---
    ih_s_alpha = filter_s_alpha.filter(demod_s_alpha)
    ih_c_alpha = filter_c_alpha.filter(demod_c_alpha)
    ih_s_beta  = filter_s_beta.filter(demod_s_beta)
    ih_c_beta  = filter_c_beta.filter(demod_c_beta)

    # --- 信号の再構成 ---
    # ×2 して実数部＝I, 虚数部＝Q の複素電流
    Ih_alpha = 2 * (ih_s_alpha + 1j*ih_c_alpha)
    Ih_beta  = 2 * (ih_s_beta  + 1j*ih_c_beta)

    # 固定 → γδ （複素ごと回転させても OK）
    Ih_gamma, Ih_delta = rot(-theta_est, np.array([Ih_alpha.real, Ih_beta.real]))

    # === (4) PLL (PI) ===
    err = Ih_delta
    int_err += err * Ts
    omega_est = Kp * err + Ki * int_err

    # === (5) 位相推定 ===
    theta_est += omega_est * Ts
    theta_est = wrap(theta_est)

    # === (0) 真の電気角更新 ===
    theta_true += omega_true * Ts
    theta_true = wrap(theta_true)

    # === ログ ===
    t_log.append(t)
    th_log.append(np.rad2deg(theta_est))
    th_true_log.append(np.rad2deg(theta_true))
    th_diff = wrap(theta_est - theta_true)
    err_deg_log.append(np.rad2deg(th_diff))

# ───────────────────────────────────────────────
# 4) 描画
# ───────────────────────────────────────────────
plt.figure(figsize=(9, 4))
plt.plot(t_log, th_log, label=r'$\hat\theta_{est}$')
plt.plot(t_log, th_true_log, label=r'$\theta_{true}$')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'Electrical angle [$^\circ$]')
plt.title(f'Phase tracking with {title_suffix}')
plt.grid(); plt.legend()

plt.figure(figsize=(9, 4))
plt.plot(t_log, err_deg_log)
plt.ylabel(r'$Phase\;error\;[deg]$')
plt.xlabel(r'$Time [s]$')
plt.title(f'Phase error convergence with {title_suffix}')
plt.grid()

plt.tight_layout()
plt.show()