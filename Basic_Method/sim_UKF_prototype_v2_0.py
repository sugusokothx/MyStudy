import numpy as np
import matplotlib.pyplot as plt

# ───────────────────────────────────────────────
# 0) モータ & フィルタ共通パラメータ
#    ─ 解析対象モータ定数と HF 注入条件 ─
# ───────────────────────────────────────────────
Va, f_h = 25.0, 1_000.0        # HF 電圧振幅 [V] と 周波数 [Hz]
ωh = 2*np.pi*f_h               # HF 角周波数 [rad/s]
Ld, Lq = 1e-3, 1.4e-3          # d/q 軸インダクタンス [H]
R_s = 0.03                     # ステータ抵抗 [Ω]
Ts, Tend = 1/6000*2, 0.05      # サンプリング周期 [s] と 総シミュレーション時間 [s]

rpm, Pn = 2000, 4
ω_true = rpm/60 * 2*np.pi * Pn # 真の電気角速度 [rad/s]（4極 2000 rpm 相当）

# ───────────────────────────────────────────────
# 1) ユーティリティ関数
# ───────────────────────────────────────────────
def wrap(th):
    """角度を -π〜π に正規化"""
    return (th + np.pi) % (2*np.pi) - np.pi

def rot(a, v):
    """2次元ベクトル v を a [rad] だけ回転"""
    c, s = np.cos(a), np.sin(a)
    return np.array([c*v[0] - s*v[1],  s*v[0] + c*v[1]])

def f_discrete(x, u):
    """
    状態遷移関数（Euler 離散化）
    x = [Id, Iq, θe, ωe]^T
    u = [Vα, Vβ]^T
    """
    Id, Iq, th, ω = x
    Vα, Vβ         = u

    # αβ → dq 変換
    Vd, Vq = rot(-th, [Vα, Vβ])

    # dq 電流微分方程式（クロスカップリング付き）を Euler 積分
    Id += Ts / Ld * (Vd - R_s*Id + ω*Lq*Iq)
    Iq += Ts / Lq * (Vq - R_s*Iq - ω*Ld*Id)

    # 角度は積分、速度は定数と仮定（ω̇=0）
    th  = wrap(th + ω*Ts)
    return np.array([Id, Iq, th, ω])

def h_meas(x):
    """観測モデル：dq 電流を αβ 軸に回転して Iαβ を生成"""
    Id, Iq, th, _ = x
    return rot(th, [Id, Iq])   # 返り値は [Iα, Iβ]

# ───────────────────────────────────────────────
# 2) UKF 用シグマ点 & 変換ルーチン
# ───────────────────────────────────────────────
n = 4                           # 状態次元
alpha, beta, kappa = 1e-3, 2.0, 0.0
lam = alpha**2 * (n + kappa) - n
gamma = np.sqrt(n + lam)        # √(n+λ)

# 重み
Wm = np.full(2*n + 1, 0.5 / (n + lam))
Wc = Wm.copy()
Wm[0] = lam / (n + lam)
Wc[0] = Wm[0] + (1 - alpha**2 + beta)

def sigma_points(x, P):
    """(2n+1) 個のシグマ点を生成"""
    S = np.linalg.cholesky(P + 1e-12*np.eye(n))  # 数値安定化項を追加
    sig = np.empty((2*n + 1, n));  sig[0] = x
    for i in range(n):
        d = gamma * S[:, i]
        sig[i + 1]     = x + d
        sig[n + i + 1] = x - d
    sig[:, 2] = np.vectorize(wrap)(sig[:, 2])    # θ 列のみ wrap
    return sig

def unscented_transform(sig, noise_cov, mean_fun=None, angle_idx=None):
    """
    シグマ点群 sig を平均・共分散に変換
    angle_idx : wrap すべき角度列（状態→2, 観測→None）
    """
    mean = (np.sum(Wm[:, None] * sig, axis=0) if mean_fun is None
            else mean_fun(sig))

    diff = sig - mean
    if angle_idx is not None:
        diff[:, angle_idx] = np.vectorize(wrap)(diff[:, angle_idx])

    cov = diff.T @ (Wc[:, None] * diff) + noise_cov
    return mean, cov, diff

# ───────────────────────────────────────────────
# 3) ノイズ共分散と初期値
# ───────────────────────────────────────────────
Q = np.diag([1e-6, 1e-6, 1e-6, 10.0])    # プロセスノイズ
R = np.diag([1e-4, 1e-4])                # 観測ノイズ

x_true = np.array([0, 0, 0, ω_true])                 # 真値初期状態
x_est  = np.array([0, 0, np.deg2rad(30), 0])         # 推定初期状態（30° オフセット）
P      = np.diag([1e-3, 1e-3, (np.pi/2)**2, 100**2]) # 初期誤差共分散

rng = np.random.default_rng(0)           # 再現性確保用乱数

# ───────────────────────────────────────────────
# 4) メインループ（UKF 推定）
# ───────────────────────────────────────────────
t_log, th_true_log, th_est_log, err_log = [], [], [], []
steps = int(Tend / Ts)

for k in range(steps):
    t = k * Ts

    # ── HF 電圧生成：推定角基準の γδ → αβ 変換 ──
    Vγ = Va * np.sin(ωh * t);  Vδ = 0.0
    Vαβ = rot(x_est[2], [Vγ, Vδ])

    # ── 真値システムを 1 ステップ進める ──
    x_true = f_discrete(x_true, Vαβ)
    y = h_meas(x_true) + rng.normal(0, np.sqrt(R[0,0]), 2)  # 測定雑音付加

    # ── UKF 予測ステップ ──
    sig    = sigma_points(x_est, P)                 # シグマ点生成
    sig_f  = np.array([f_discrete(s, Vαβ) for s in sig])  # 各点を伝搬
    x_pred, P_pred, diff_f = unscented_transform(sig_f, Q, angle_idx=2)

    # ── UKF 更新ステップ ──
    sig_h  = np.array([h_meas(s) for s in sig_f])   # 観測空間へ射影
    y_pred, Pyy, diff_h = unscented_transform(
        sig_h, R,
        mean_fun=lambda s: np.sum(Wm[:, None] * s, axis=0),
        angle_idx=None)                             # 観測側は wrap 不要

    Pxy = diff_f.T @ (Wc[:, None] * diff_h)         # 状態-観測クロス共分散
    K   = Pxy @ np.linalg.inv(Pyy)                  # カルマンゲイン

    x_est = x_pred + K @ (y - y_pred)               # 状態更新
    x_est[2] = wrap(x_est[2])                       # 角度の正規化
    P = P_pred - K @ Pyy @ K.T                      # 分散更新

    # ── ログ格納 ──
    t_log.append(t)
    th_true_log.append(np.rad2deg(x_true[2]))
    th_est_log .append(np.rad2deg(x_est [2]))
    err_log    .append(np.rad2deg(wrap(x_est[2] - x_true[2])))

# ───────────────────────────────────────────────
# 5) 可視化
# ───────────────────────────────────────────────
plt.figure(figsize=(10, 4))
plt.plot(t_log, th_true_log, label='θ true')
plt.plot(t_log, th_est_log, '--', label='θ UKF')
plt.ylabel('Electrical angle [deg]')
plt.xlabel('Time [s]')
plt.title('UKF phase tracking')
plt.grid(); plt.legend()

plt.figure(figsize=(10, 3))
plt.plot(t_log, err_log)
plt.ylabel('Error [deg]')
plt.xlabel('Time [s]')
plt.title('Phase estimation error (UKF)')
plt.grid()
plt.tight_layout(); plt.show()
