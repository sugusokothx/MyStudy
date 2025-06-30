import numpy as np
from scipy.interpolate import RegularGridInterpolator, griddata
import matplotlib.pyplot as plt

# ────────────────────────────────────────────────────────────
# 0)  実機マップとパラメータ定義
# ────────────────────────────────────────────────────────────
# --- モータパラメータ ---
Pn = 4                   # 対数
Rs_true = 0.05           # 固定子抵抗 [Ω] (真値)
Ld_nom, Lq_nom = 0.3e-3, 0.5e-3 # 名目インダクタンス [H]
psi_f_nom = 0.08         # 名目磁石鎖交磁束 [Wb]
DT = 1e-4                # 制御周期 [s]

# --- 順マップ (Id,Iq -> Phi_d,Phi_q) の生成 (EKFが使用) ---
IdAxis = np.linspace(-150, 150, 61)
IqAxis = np.linspace(-200, 400, 61)
Id_mesh, Iq_mesh = np.meshgrid(IdAxis, IqAxis, indexing='ij')

# 簡単な線形モデルでマップを生成（実機ではここを実測値で置換）
PhiDMap = Ld_nom * Id_mesh + psi_f_nom
PhiQMap = Lq_nom * Iq_mesh

phi_d_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiDMap,
                                    bounds_error=False, fill_value=None)
phi_q_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiQMap,
                                    bounds_error=False, fill_value=None)

# --- 逆マップ (Phi_d,Phi_q -> Id,Iq) の生成 (MotorModelが使用) ---
# 順マップのデータ点から逆引きしてテーブルを生成する
points = np.array([PhiDMap.flatten(), PhiQMap.flatten()]).T
values_id = Id_mesh.flatten()
values_iq = Iq_mesh.flatten()

# 逆引き先のグリッドを定義
psi_d_axis = np.linspace(np.min(PhiDMap), np.max(PhiDMap), 61)
psi_q_axis = np.linspace(np.min(PhiQMap), np.max(PhiQMap), 61)
grid_psi_d, grid_psi_q = np.meshgrid(psi_d_axis, psi_q_axis, indexing='ij')

# griddataで逆引き計算
IdTable = griddata(points, values_id, (grid_psi_d, grid_psi_q), method='linear')
IqTable = griddata(points, values_iq, (grid_psi_d, grid_psi_q), method='linear')

# グリッド外のNaNを最近傍の値で埋める
IdTable = griddata(points, values_id, (grid_psi_d, grid_psi_q), method='nearest')
IqTable = griddata(points, values_iq, (grid_psi_d, grid_psi_q), method='nearest')


# ────────────────────────────────────────────────────────────
# 1)  プラントモデル (MATLABコードをPythonに変換)
# ────────────────────────────────────────────────────────────
class MotorModel:
    """
    MATLABコード `pmsm_dq_model` をPythonに変換したプラントモデル。
    内部で磁束(psi_d, psi_q)を状態として保持する。
    """
    def __init__(self, Ts, Pn, Rs, psi_d_axis, psi_q_axis, IdTable, IqTable):
        self.Ts = Ts
        self.Pn = Pn
        self.Rs = Rs
        self.psi_d = 0.0
        self.psi_q = 0.0

        # 磁束から電流を求めるための2D補間器を初期化
        self.id_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IdTable,
                                              bounds_error=False, fill_value=None)
        self.iq_lut = RegularGridInterpolator((psi_d_axis, psi_q_axis), IqTable,
                                              bounds_error=False, fill_value=None)
    
    def step(self, u, dpsi_f_true=0.0):
        """
        1ステップ計算を進める
        u: [v_d, v_q, omega]
        dpsi_f_true: 温度上昇による真の磁束弱化量
        """
        v_d, v_q, omega = u

        # 現在の磁束から電流を計算
        # dpsi_f_trueを考慮して真の磁束を計算
        psi_d_with_temp_effect = self.psi_d - dpsi_f_true
        
        i_d = self.id_lut([psi_d_with_temp_effect, self.psi_q])
        i_q = self.iq_lut([psi_d_with_temp_effect, self.psi_q])

        # 磁束の微分を計算 (dψ/dt = v - R·i + ω·J·ψ)
        dpsi_d = v_d - self.Rs * i_d + omega * self.psi_q
        dpsi_q = v_q - self.Rs * i_q - omega * self.psi_d

        # オイラー法で磁束を更新
        self.psi_d += self.Ts * dpsi_d
        self.psi_q += self.Ts * dpsi_q
        
        # トルクを計算
        Te = 1.5 * self.Pn * (psi_d_with_temp_effect * i_q - self.psi_q * i_d)

        return np.array([i_d, i_q, Te])


# ────────────────────────────────────────────────────────────
# 2)  Δφ-EKF (+Δψ_f) 本体 (変更なし)
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
                 Q_diag=(1e-6, 1e-6,   1e-8, 1e-8, 1e-9, 1e-9),
                 R_diag=(1e-4, 1e-4)):
        self.dt = dt
        self.Ld = Ld
        self.Lq = Lq
        self.psi_f_nom = psi_f_nom
        self.x_hat = np.zeros((6, 1))
        # Rsの初期値を真値と少しずらしておく
        self.x_hat[5, 0] = Rs_true * 0.8 
        self.P = np.diag([1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4])
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.H = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])

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
        return float(phi_d_lut([self.i_d, self.i_q])) + self.delta_phi_d + self.delta_psi_f
    @property
    def phi_q(self):
        return float(phi_q_lut([self.i_d, self.i_q])) + self.delta_phi_q
    @property
    def torque(self):
        return 1.5 * Pn * (self.phi_d * self.i_q - self.phi_q * self.i_d)

# ────────────────────────────────────────────────────────────
# 3)  動作テスト (プラントモデルとEKFを連携)
# ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    
    # --- インスタンス生成 ---
    motor = MotorModel(DT, Pn, Rs_true, psi_d_axis, psi_q_axis, IdTable, IqTable)
    ekf = DeltaPhiEKF_B(DT, Ld_nom, Lq_nom)

    # --- シミュレーション条件 ---
    v_d_cmd, v_q_cmd = 20.0, 100.0  # 入力電圧指令 [V]
    ω_e = 2 * np.pi * 150         # 電気角速度 [rad/s] (150 Hz)
    u_cmd = (v_d_cmd, v_q_cmd, ω_e)

    # --- データ記録用リスト ---
    history = []
    
    # --- 温度上昇を模擬：真の磁石磁束をゆっくり弱化させる ---
    dpsi_f_true = 0.0

    print("シミュレーション開始...")
    for k in range(30000):  # 3秒間のシミュレーション
        
        # ----- (1) 真の磁束弱化量を変化させる -----
        # 1秒後から2秒後にかけて、磁束が10%リニアに減少すると仮定
        if 10000 <= k < 20000:
            dpsi_f_true -= 0.10 * (psi_f_nom / 10000)

        # ----- (2) プラントモデルを1ステップ進める -----
        i_d_true, i_q_true, Te_true = motor.step(u_cmd, dpsi_f_true)

        # ----- (3) 観測値を生成（真値にノイズを付加） -----
        obs_noise_std = np.sqrt(ekf.R.diagonal()) # EKFのRと合わせる
        z_meas = np.array([i_d_true, i_q_true]) + \
                 np.random.normal(0, obs_noise_std, 2)
        
        # ----- (4) EKFの予測と更新 -----
        ekf.predict(u_cmd)
        ekf.update(z_meas)

        # ----- (5) データ保存 -----
        record = {
            "t": k * DT,
            "i_d_true": i_d_true, "i_d_est": ekf.i_d,
            "i_q_true": i_q_true, "i_q_est": ekf.i_q,
            "Rs_true": Rs_true,   "Rs_est": ekf.R_s,
            "dpsi_f_true": dpsi_f_true, "dpsi_f_est": ekf.delta_psi_f,
            "Te_true": Te_true, "Te_est": ekf.torque
        }
        history.append(record)

    print("シミュレーション完了。グラフを描画します。")

    # --- グラフ描画 ---
    import pandas as pd
    df = pd.DataFrame(history)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('EKFによるモータパラメータ・トルク推定の性能評価', fontsize=16)

    axes[0].plot(df['t'], df['dpsi_f_true'], 'r--', label='Δψ_f (真値)')
    axes[0].plot(df['t'], df['dpsi_f_est'], 'b-', label='Δψ_f (推定値)')
    axes[0].set_ylabel('磁束弱化量 [Wb]')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(df['t'], df['Rs_true'], 'r--', label='R_s (真値)')
    axes[1].plot(df['t'], df['Rs_est'], 'b-', label='R_s (推定値)')
    axes[1].set_ylabel('固定子抵抗 [Ω]')
    axes[1].legend()
    axes[1].grid(True)
    
    axes[2].plot(df['t'], df['i_d_true'], 'r:', alpha=0.7, label='I_d (真値)')
    axes[2].plot(df['t'], df['i_d_est'], 'b-', alpha=0.9, label='I_d (推定値)')
    axes[2].plot(df['t'], df['i_q_true'], 'm:', alpha=0.7, label='I_q (真値)')
    axes[2].plot(df['t'], df['i_q_est'], 'c-', alpha=0.9, label='I_q (推定値)')
    axes[2].set_ylabel('d-q電流 [A]')
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(df['t'], df['Te_true'], 'r--', label='Torque (真値)')
    axes[3].plot(df['t'], df['Te_est'], 'b-', label='Torque (推定値)')
    axes[3].set_xlabel('Time [s]')
    axes[3].set_ylabel('トルク [Nm]')
    axes[3].legend()
    axes[3].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()