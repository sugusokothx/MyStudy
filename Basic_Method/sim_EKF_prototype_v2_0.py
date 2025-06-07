import numpy as np
import matplotlib.pyplot as plt

# ───────────────────────────────────────────────
# 0)  固定パラメータ
# ───────────────────────────────────────────────
Va = 25.0          # HF 電圧振幅 [V]
f_h = 1_000.0      # HF 周波数 [Hz]
ωh  = 2*np.pi*f_h

Ld, Lq = 1e-3, 1.4e-3   # [H]
R_s = 0.03              # [Ω]

Ts   = 1/6000 * 2       # [s]
Tend = 0.05             # [s]

# 真の速度（4 極, 2000 rpm）
rpm, Pn = 2000, 4
ω_true = rpm/60 * 2*np.pi * Pn

# EKF ― 共分散初期値・ノイズ
P = np.diag([1e-3, 1e-3, (np.pi/2)**2, (100.0)**2])
Q = np.diag([1e-6, 1e-6, 1e-6, 10.0])     # プロセス
R = np.diag([1e-4, 1e-4])                 # 観測

rng = np.random.default_rng(0)            # 再現用

# ───────────────────────────────────────────────
# 1)  ユーティリティ
# ───────────────────────────────────────────────
def wrap(th):                # −π〜π
    return (th + np.pi) % (2*np.pi) - np.pi

def rot(alpha, vec):         # 2D 回転
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([ c*vec[0] - s*vec[1],
                      s*vec[0] + c*vec[1] ])

def f_discrete(x, u):
    """状態遷移（Euler 離散）"""
    Id, Iq, th, ω = x
    Vα, Vβ = u
    Vd, Vq = rot(-th, [Vα, Vβ])

    dId = (Vd - R_s*Id + ω*Lq*Iq) / Ld
    dIq = (Vq - R_s*Iq - ω*Ld*Id) / Lq
    dth = ω
    return np.array([Id + dId*Ts,
                     Iq + dIq*Ts,
                     wrap(th + dth*Ts),
                     ω])           # ω̇=0

def h_measure(x):
    """観測モデル：Iαβ"""
    Id, Iq, th, _ = x
    return rot(th, [Id, Iq])

def jacobian(fun, x, eps=1e-6):
    """数値 Jacobian"""
    f0 = fun(x)
    m, n = f0.size, x.size
    J = np.zeros((m, n))
    for i in range(n):
        dx = np.zeros(n); dx[i] = eps
        J[:, i] = (fun(x+dx) - fun(x-dx)) / (2*eps)
    return J

# ───────────────────────────────────────────────
# 2)  初期状態
# ───────────────────────────────────────────────
x_true = np.array([0.0, 0.0, 0.0,          ω_true])   # ground-truth
x_est  = np.array([0.0, 0.0, np.deg2rad(30), 0.0])    # EKF 初期推定

# ───────────────────────────────────────────────
# 3)  シミュレーション / EKF ループ
# ───────────────────────────────────────────────
t_log, th_true_log, th_est_log, err_log = [], [], [], []
steps = int(Tend / Ts)

for k in range(steps):
    t = k * Ts

    # --- HF 電圧（γδ 軸で sin 波） ---
    Vγ = Va * np.sin(ωh * t)
    Vδ = 0.0
    Vαβ = rot(x_est[2], [Vγ, Vδ])          # 実機では推定角で回転

    # ---------- 真値系を 1step 進める ----------
    x_true = f_discrete(x_true, Vαβ)

    # 観測（雑音付き）
    y = h_measure(x_true) + rng.normal(0, np.sqrt(R[0,0]), size=2)

    # ---------- EKF 予測 ----------
    f = lambda x: f_discrete(x, Vαβ)
    x_pred = f(x_est)
    F = jacobian(f, x_est)
    P_pred = F @ P @ F.T + Q

    # ---------- EKF 更新 ----------
    H = jacobian(h_measure, x_pred)
    y_pred = h_measure(x_pred)
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x_est = x_pred + K @ (y - y_pred)
    x_est[2] = wrap(x_est[2])              # 角度正規化
    P = (np.eye(4) - K @ H) @ P_pred

    # ---------- ログ ----------
    t_log.append(t)
    th_true_log.append(np.rad2deg(wrap(x_true[2])))
    th_est_log .append(np.rad2deg(wrap(x_est [2])))
    err_log.append(np.rad2deg(wrap(x_est[2] - x_true[2])))

# ───────────────────────────────────────────────
# 4)  可視化
# ───────────────────────────────────────────────
plt.figure(figsize=(10,4))
plt.plot(t_log, th_true_log, label='θ true')
plt.plot(t_log, th_est_log , '--', label='θ EKF')
plt.ylabel('Electrical angle [deg]'); plt.xlabel('Time [s]')
plt.title('EKF phase tracking'); plt.grid(); plt.legend()

plt.figure(figsize=(10,3))
plt.plot(t_log, err_log)
plt.ylabel('Error [deg]'); plt.xlabel('Time [s]')
plt.title('Phase estimation error'); plt.grid()
plt.tight_layout(); plt.show()
