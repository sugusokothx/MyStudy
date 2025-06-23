import numpy as np
from scipy.interpolate import RegularGridInterpolator, griddata
import matplotlib.pyplot as plt
import pandas as pd

# ────────────────────────────────────────────────────────────
# 0)  実機マップとパラメータ定義
# ────────────────────────────────────────────────────────────
Pn = 4                                # 極対数
Rs_true = 0.05                        # 固定子抵抗 [Ω]
Ld_nom, Lq_nom = 0.3e-3, 0.5e-3       # 名目インダクタンス [H]
psi_f_nom = 0.08                      # 名目磁石鎖交磁束 [Wb]
DT = 1e-4                             # 制御周期 [s]

# --- 順マップ (Id,Iq → Φd,Φq) --------------------------------
IdAxis = np.linspace(-150, 150, 61)
IqAxis = np.linspace(-200, 400, 61)
Id_mesh, Iq_mesh = np.meshgrid(IdAxis, IqAxis, indexing='ij')

PhiDMap = Ld_nom * Id_mesh + psi_f_nom
PhiQMap = Lq_nom * Iq_mesh

phi_d_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiDMap,
                                    bounds_error=False, fill_value=None)
phi_q_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiQMap,
                                    bounds_error=False, fill_value=None)

# --- 逆マップ (Φd,Φq → Id,Iq) --------------------------------
points = np.array([PhiDMap.ravel(), PhiQMap.ravel()]).T
values_id = Id_mesh.ravel()
values_iq = Iq_mesh.ravel()

psi_d_axis = np.linspace(PhiDMap.min(), PhiDMap.max(), 61)
psi_q_axis = np.linspace(PhiQMap.min(), PhiQMap.max(), 61)
grid_psi_d, grid_psi_q = np.meshgrid(psi_d_axis, psi_q_axis, indexing='ij')

# 最近傍補間だけで十分
IdTable = griddata(points, values_id,
                   (grid_psi_d, grid_psi_q), method='nearest')
IqTable = griddata(points, values_iq,
                   (grid_psi_d, grid_psi_q), method='nearest')

# ────────────────────────────────────────────────────────────
# 1)  プラントモデル
# ────────────────────────────────────────────────────────────
class MotorModel:
    def __init__(self, Ts, Pn, Rs,
                 psi_d_axis, psi_q_axis, IdTable, IqTable):
        self.Ts, self.Pn, self.Rs = Ts, Pn, Rs
        self.psi_d = 0.0
        self.psi_q = 0.0
        self.id_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IdTable,
                                              bounds_error=False, fill_value=None)
        self.iq_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IqTable,
                                              bounds_error=False, fill_value=None)

    def step(self, u, dpsi_f_true=0.0):
        v_d, v_q, omega = u

        # 実際の Φd に弱化分を反映
        psi_d_with_temp = self.psi_d - dpsi_f_true

        # LUT は ndarray(1,) を返すので float 化
        i_d = float(self.id_lut([psi_d_with_temp, self.psi_q]))
        i_q = float(self.iq_lut([psi_d_with_temp, self.psi_q]))

        # dψ/dt = v - R·i + ω·J·ψ
        dpsi_d = v_d - self.Rs * i_d + omega * self.psi_q
        dpsi_q = v_q - self.Rs * i_q - omega * self.psi_d

        # オイラー更新
        self.psi_d += self.Ts * dpsi_d
        self.psi_q += self.Ts * dpsi_q

        Te = 1.5 * self.Pn * (psi_d_with_temp * i_q - self.psi_q * i_d)
        return np.array([i_d, i_q, Te])

# ────────────────────────────────────────────────────────────
# 2)  Δφ-EKF (+Δψ_f) 本体（B 案）
# ────────────────────────────────────────────────────────────
def numerical_jacobian(f, x, u, eps=1e-7):
    n = len(x)
    Fx = np.zeros((n, n))
    f0 = f(x, u)
    for k in range(n):
        x1 = x.copy()
        x1[k] += eps
        Fx[:, k] = (f(x1, u) - f0) / eps
    return Fx

class DeltaPhiEKF_B:
    def __init__(self, dt, Ld, Lq,
                 Q_diag=(1e-6, 1e-6, 1e-8, 1e-8, 1e-9, 1e-9),
                 R_diag=(1e-4, 1e-4)):
        self.dt, self.Ld, self.Lq = dt, Ld, Lq
        self.psi_f_nom = psi_f_nom

        self.x_hat = np.zeros((6, 1))
        self.x_hat[5, 0] = Rs_true * 0.8          # R_s 初期値をずらす
        self.P = np.diag([1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4])
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0]])

    def _f_continuous(self, x, u):
        i_d, i_q, dphi_d, dphi_q, dpsi_f, R_s = x
        v_d, v_q, ω_e = u

        λ_d = self.Ld * i_d + self.psi_f_nom + dpsi_f + dphi_d
        λ_q = self.Lq * i_q + dphi_q

        di_d = (v_d + ω_e * λ_q - R_s * i_d) / self.Ld
        di_q = (v_q - ω_e * λ_d - R_s * i_q) / self.Lq
        return np.array([di_d, di_q, 0.0, 0.0, 0.0, 0.0])

    def predict(self, u):
        f_val = self._f_continuous(self.x_hat.flatten(), u)
        self.x_hat += f_val.reshape(-1, 1) * self.dt
        F = numerical_jacobian(self._f_continuous, self.x_hat.flatten(), u)
        F_k = np.eye(6) + F * self.dt
        self.P = F_k @ self.P @ F_k.T + self.Q

    def update(self, z):
        z = z.reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x_hat
        self.x_hat += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    # --- アクセサ ---
    @property
    def i_d(self): return self.x_hat[0, 0]
    @property
    def i_q(self): return self.x_hat[1, 0]
    @property
    def delta_phi_d(self): return self.x_hat[2, 0]
    @property
    def delta_phi_q(self): return self.x_hat[3, 0]
    @property
    def delta_psi_f(self): return self.x_hat[4, 0]
    @property
    def R_s(self): return self.x_hat[5, 0]

    @property
    def phi_d(self):
        return float(phi_d_lut([self.i_d, self.i_q])) \
               + self.delta_phi_d + self.delta_psi_f

    @property
    def phi_q(self):
        return float(phi_q_lut([self.i_d, self.i_q])) \
               + self.delta_phi_q

    @property
    def torque(self):
        return 1.5 * Pn * (self.phi_d * self.i_q - self.phi_q * self.i_d)

# ────────────────────────────────────────────────────────────
# 3)  動作テスト
# ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    motor = MotorModel(DT, Pn, Rs_true,
                       psi_d_axis, psi_q_axis, IdTable, IqTable)
    ekf = DeltaPhiEKF_B(DT, Ld_nom, Lq_nom)

    v_d_cmd, v_q_cmd = 20.0, 100.0
    ω_e = 2 * np.pi * 150
    u_cmd = (v_d_cmd, v_q_cmd, ω_e)

    # ノイズ標準偏差（ループ外で 1 回取得）
    obs_noise_std = np.sqrt(np.diag(ekf.R))

    history = []
    dpsi_f_true = 0.0

    print("シミュレーション開始...")
    for k in range(30000):  # 3 s
        # 磁束弱化 1–2 s 区間で 10 %
        if 10000 <= k < 20000:
            dpsi_f_true -= 0.10 * (psi_f_nom / 10000)

        i_d_true, i_q_true, Te_true = motor.step(u_cmd, dpsi_f_true)

        z_meas = np.array([i_d_true, i_q_true]) \
                 + np.random.normal(0, obs_noise_std, 2)

        ekf.predict(u_cmd)
        ekf.update(z_meas)

        history.append({
            "t": k * DT,
            "i_d_true": i_d_true, "i_d_est": ekf.i_d,
            "i_q_true": i_q_true, "i_q_est": ekf.i_q,
            "Rs_true": Rs_true,   "Rs_est": ekf.R_s,
            "dpsi_f_true": dpsi_f_true, "dpsi_f_est": ekf.delta_psi_f,
            "Te_true": Te_true, "Te_est": ekf.torque
        })

    print("シミュレーション完了。グラフを描画します。")

    df = pd.DataFrame(history)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('EKF によるモータパラメータ・トルク推定', fontsize=16)

    axes[0].plot(df['t'], df['dpsi_f_true'], 'r--', label='Δψ_f 真値')
    axes[0].plot(df['t'], df['dpsi_f_est'], 'b-', label='Δψ_f 推定')
    axes[0].set_ylabel('磁束弱化量 [Wb]')
    axes[0].legend(); axes[0].grid(True)

    axes[1].plot(df['t'], df['Rs_true'], 'r--', label='R_s 真値')
    axes[1].plot(df['t'], df['Rs_est'], 'b-', label='R_s 推定')
    axes[1].set_ylabel('固定子抵抗 [Ω]')
    axes[1].legend(); axes[1].grid(True)

    axes[2].plot(df['t'], df['i_d_true'], 'r:', label='I_d 真値')
    axes[2].plot(df['t'], df['i_d_est'], 'b-', label='I_d 推定')
    axes[2].plot(df['t'], df['i_q_true'], 'm:', label='I_q 真値')
    axes[2].plot(df['t'], df['i_q_est'], 'c-', label='I_q 推定')
    axes[2].set_ylabel('d-q 電流 [A]')
    axes[2].legend(); axes[2].grid(True)

    axes[3].plot(df['t'], df['Te_true'], 'r--', label='Torque 真値')
    axes[3].plot(df['t'], df['Te_est'], 'b-', label='Torque 推定')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('トルク [Nm]')
    axes[3].legend(); axes[3].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()
