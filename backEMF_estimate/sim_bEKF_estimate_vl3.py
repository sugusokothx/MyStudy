import numpy as np
from scipy.interpolate import RegularGridInterpolator

# ────────────────────────────────────────────────────────────
# 0)  実機マップをここに流し込む（★サンプルではダミー値を生成）
# ────────────────────────────────────────────────────────────
#   ※ 実運用では IdAxis/IqAxis, PhiDMap, PhiQMap を実測値に置換してください
IdAxis = np.linspace(-150, 150, 61)                    # [A]
IqAxis = np.linspace(-200, 400, 61)                    # [A]
Id_mesh, Iq_mesh = np.meshgrid(IdAxis, IqAxis, indexing='ij')
Ld_nom, Lq_nom = 0.3e-3, 0.5e-3                        # 名目インダクタンス
psi_f_nom = 0.08                                       # 名目磁石鎖交磁束 [Wb]

PhiDMap = Ld_nom * Id_mesh + psi_f_nom                 # φd ≈ Ld·Id + ψf_nom
PhiQMap = Lq_nom * Iq_mesh                             # φq ≈ Lq·Iq

phi_d_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiDMap,
                                    bounds_error=False, fill_value=None)
phi_q_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiQMap,
                                    bounds_error=False, fill_value=None)

# ────────────────────────────────────────────────────────────
# 1)  数値ヤコビアン
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

# ────────────────────────────────────────────────────────────
# 2)  Δφ-EKF (+Δψ_f) 本体
# ────────────────────────────────────────────────────────────
class DeltaPhiEKF_B:
    r"""
    状態 x = [ i_d, i_q, Δφ_d, Δφ_q, Δψ_f, R_s ]ᵀ
      ・Δφ_d, Δφ_q : マップからのずれ （インダクタンス変動由来）
      ・Δψ_f        : 永久磁石磁束の弱化分
      ・R_s         : 固定子抵抗
    観測 z = [ i_d_meas, i_q_meas ]ᵀ
    """

    def __init__(self, dt, Ld, Lq,
                 Q_diag=(1e-6, 1e-6,   1e-8, 1e-8, 1e-9, 1e-9),
                 R_diag=(1e-4, 1e-4)):
        self.dt   = dt
        self.Ld   = Ld
        self.Lq   = Lq
        self.psi_f_nom = psi_f_nom              # 名目磁石鎖交磁束

        # ------ 推定値と共分散の初期化 ------
        self.x_hat = np.zeros((6, 1))           # 6×1 ベクトル
        self.P     = np.diag([1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4])

        self.Q = np.diag(Q_diag)                # プロセスノイズ
        self.R = np.diag(R_diag)                # 観測ノイズ
        self.H = np.array([[1, 0, 0, 0, 0, 0],  # 観測行列 (i_d, i_q のみ)
                           [0, 1, 0, 0, 0, 0]])

    # ---------- 連続時間モデル f(x,u) ----------
    def _f_continuous(self, x, u):
        """
        u = (v_d, v_q, ω_e)  [V, V, rad/s]
        """
        i_d, i_q, dphi_d, dphi_q, dpsi_f, R_s = x
        v_d, v_q, ω_e = u

        # λ_d, λ_q を構成
        λ_d = self.Ld * i_d + self.psi_f_nom + dpsi_f + dphi_d
        λ_q = self.Lq * i_q + dphi_q

        # dq 電圧式を解いて di/dt
        di_d = (v_d + ω_e * λ_q - R_s * i_d) / self.Ld
        di_q = (v_q - ω_e * λ_d - R_s * i_q) / self.Lq

        # Δφ_d, Δφ_q, Δψ_f, R_s はランダムウォーク
        return np.array([di_d, di_q, 0.0, 0.0, 0.0, 0.0])

    # ---------- EKF 予測 ----------
    def predict(self, u):
        f_val = self._f_continuous(self.x_hat.flatten(), u)
        self.x_hat += f_val.reshape(-1, 1) * self.dt

        F = numerical_jacobian(self._f_continuous,
                               self.x_hat.flatten(), u)
        F_k = np.eye(6) + F * self.dt
        self.P = F_k @ self.P @ F_k.T + self.Q

    # ---------- EKF 更新 ----------
    def update(self, z):
        z = z.reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x_hat
        self.x_hat += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    # ---------- 推定値アクセサ ----------
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

    # φd, φq = マップ値 + Δφ + Δψ_f (d 軸のみ)
    @property
    def phi_d(self):
        return float(phi_d_lut([self.i_d, self.i_q])) \
               + self.delta_phi_d + self.delta_psi_f
    @property
    def phi_q(self):
        return float(phi_q_lut([self.i_d, self.i_q])) \
               + self.delta_phi_q

    @property
    def torque(self, pole_pairs=4):
        return 1.5 * pole_pairs * (self.phi_d * self.i_q -
                                   self.phi_q * self.i_d)

# ────────────────────────────────────────────────────────────
# 3)  動作テスト
# ────────────────────────────────────────────────────────────
if __name__ == "__main__":

    DT = 1e-4
    ekf = DeltaPhiEKF_B(DT, Ld_nom, Lq_nom)

    # 入力電圧（固定） & 電気角速度
    v_d_cmd, v_q_cmd = 20.0, 60.0     # [V]
    ω_e = 2*np.pi*150                 # 150 Hz

    # 温度上昇を模擬：磁石弱化を 2 %／秒 で進行させる
    dpsi_true = 0.0

    for k in range(30000):            # 3 秒
        # ----- 真の Δψ_f をゆっくり変化させる -----
        dpsi_true -= 0.02 * DT * psi_f_nom   # 2 % /s 減少

        # ---- 真の λ_d, λ_q, i_d, i_q を生成 (簡易: 定常値仮定) ----
        # 実際はモータの電流応答をシミュレートすべきだが，
        # ここでは “測定値 = 推定値 + 測定ノイズ” として手軽に試す
        z_meas = np.array([ekf.i_d, ekf.i_q]) + \
                 np.random.normal(0, np.sqrt(1e-4), 2)

        # EKF ステップ
        ekf.predict((v_d_cmd, v_q_cmd, ω_e))
        ekf.update(z_meas)

        # ----- デバッグ表示 -----
        if k % 2000 == 0:
            print(f"k={k:5d}  Δψ_f_est={ekf.delta_psi_f:+8.5f}  "
                  f"Δψ_f_true={dpsi_true:+8.5f}  "
                  f"Te={ekf.torque:+6.3f} Nm")
