import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import cholesky

# --- 1. IPMSM（シミュレータ）クラスの定義 (前回と同じ) ---
class IPMSM_Simulator:
    """
    IPMモータの実際の動作をシミュレートするクラス。
    温度上昇によるパラメータ変動を再現する。
    """
    def __init__(self, dt, Ld, Lq, pole_pairs):
        self.dt = dt
        self.Ld = Ld
        self.Lq = Lq
        self.pole_pairs = pole_pairs
        self.temp_rotor_C = 25.0
        self.R_s_true = 0.05
        self.psi_f_true = 0.1
        self.i_d = 0.0
        self.i_q = 0.0
        self.alpha_R = 0.00393
        self.alpha_psi = -0.0008

    def update_true_parameters(self, t):
        if self.temp_rotor_C < 85.0:
            self.temp_rotor_C = 25.0 + t
        self.R_s_true = 0.05 * (1 + self.alpha_R * (self.temp_rotor_C - 25.0))
        self.psi_f_true = 0.1 * (1 + self.alpha_psi * (self.temp_rotor_C - 25.0))

    def step(self, v_d, v_q, we):
        di_d_dt = (1/self.Ld) * (-self.R_s_true * self.i_d + we * self.Lq * self.i_q + v_d)
        di_q_dt = (1/self.Lq) * (-self.R_s_true * self.i_q - we * self.Ld * self.i_d - we * self.psi_f_true + v_q)
        self.i_d += di_d_dt * self.dt
        self.i_q += di_q_dt * self.dt
        return self.i_d, self.i_q

# --- 2. アンセンテッドカルマンフィルタ（UKF）クラスの定義 ---
class UnscentedKalmanFilter:
    """
    固定子抵抗と磁束を推定するためのUKF
    状態ベクトル x = [i_d, i_q, R_s, psi_f]^T
    """
    def __init__(self, dt, Ld, Lq, pole_pairs, n_states):
        self.dt = dt
        self.Ld = Ld
        self.Lq = Lq
        self.n_states = n_states
        
        # UKFパラメータ
        self.alpha = 1e-3
        self.kappa = 0
        self.beta = 2
        self.lambda_ = self.alpha**2 * (self.n_states + self.kappa) - self.n_states
        
        # シグマポイントの重み
        self.wm = np.full(2 * self.n_states + 1, 1 / (2 * (self.n_states + self.lambda_)))
        self.wc = np.full(2 * self.n_states + 1, 1 / (2 * (self.n_states + self.lambda_)))
        self.wm[0] = self.lambda_ / (self.n_states + self.lambda_)
        self.wc[0] = self.lambda_ / (self.n_states + self.lambda_) + (1 - self.alpha**2 + self.beta)

        # 状態ベクトル x = [i_d, i_q, R_s, psi_f]^T の初期推定値
        self.x_hat = np.array([0.0, 0.0, 0.04, 0.11]).reshape(self.n_states, 1)

        # 誤差共分散行列 P の初期値
        self.P = np.diag([1e-3, 1e-3, 1e-4, 1e-4])

        # プロセスノイズ共分散行列 Q
        self.Q = np.diag([1e-5, 1e-5, 1e-9, 1e-10])

        # 観測ノイズ共分散行列 R
        self.R = np.diag([1e-4, 1e-4])

    def _generate_sigma_points(self):
        """シグマポイントを生成する"""
        sigma_points = np.zeros((self.n_states, 2 * self.n_states + 1))
        
        # Pの平方根を計算 (コレスキー分解)
        try:
            P_sqrt = cholesky((self.n_states + self.lambda_) * self.P)
        except np.linalg.LinAlgError:
            # Pが半正定値でない場合、小さな値を対角成分に加えて再試行
            P_sqrt = cholesky((self.n_states + self.lambda_) * (self.P + 1e-6*np.eye(self.n_states)))

        sigma_points[:, 0] = self.x_hat.flatten()
        for i in range(self.n_states):
            sigma_points[:, i + 1] = (self.x_hat + P_sqrt[:, i].reshape(self.n_states, 1)).flatten()
            sigma_points[:, i + 1 + self.n_states] = (self.x_hat - P_sqrt[:, i].reshape(self.n_states, 1)).flatten()
        return sigma_points

    def _motor_dynamics(self, x, u):
        """非線形の状態方程式（状態遷移関数）"""
        v_d, v_q, we = u
        i_d, i_q, R_s, psi_f = x
        
        # 連続時間モデル
        x_dot = np.array([
            (1/self.Ld) * (-R_s * i_d + we * self.Lq * i_q + v_d),
            (1/self.Lq) * (-R_s * i_q - we * self.Ld * i_d - we * psi_f + v_q),
            0,
            0
        ])
        # 離散化
        return x + x_dot * self.dt

    def predict(self, u):
        """予測ステップ"""
        # 1. シグマポイントを生成
        sigma_points = self._generate_sigma_points()
        
        # 2. 各シグマポイントを状態方程式に通す
        sigma_points_pred = np.zeros_like(sigma_points)
        for i in range(2 * self.n_states + 1):
            sigma_points_pred[:, i] = self._motor_dynamics(sigma_points[:, i], u)
        
        # 3. 予測状態ベクトルを計算
        self.x_hat = np.sum(self.wm * sigma_points_pred, axis=1).reshape(self.n_states, 1)

        # 4. 予測誤差共分散行列を計算
        P_pred = np.zeros((self.n_states, self.n_states))
        for i in range(2 * self.n_states + 1):
            y = (sigma_points_pred[:, i].reshape(self.n_states, 1) - self.x_hat)
            P_pred += self.wc[i] * (y @ y.T)
        
        self.P = P_pred + self.Q
        self.sigma_points_pred = sigma_points_pred # 更新ステップで再利用

    def update(self, z):
        """更新ステップ"""
        # z: 観測ベクトル [i_d_obs, i_q_obs]^T
        z = z.reshape(2, 1)
        n_obs = len(z)
        
        # 1. 予測されたシグマポイントを観測方程式に通す
        # h(x) = [i_d, i_q]^T なので、状態ベクトルの上2つを取り出す
        Z_pred = self.sigma_points_pred[:n_obs, :]
        
        # 2. 予測観測値を計算
        z_hat = np.sum(self.wm * Z_pred, axis=1).reshape(n_obs, 1)
        
        # 3. 観測誤差共分散 S と 相互共分散 T を計算
        S = np.zeros((n_obs, n_obs))
        T = np.zeros((self.n_states, n_obs))
        for i in range(2 * self.n_states + 1):
            err_z = (Z_pred[:, i].reshape(n_obs, 1) - z_hat)
            err_x = (self.sigma_points_pred[:, i].reshape(self.n_states, 1) - self.x_hat)
            S += self.wc[i] * (err_z @ err_z.T)
            T += self.wc[i] * (err_x @ err_z.T)
        
        S += self.R
        
        # 4. カルマンゲインを計算
        K = T @ np.linalg.inv(S)

        # 5. 状態ベクトルと誤差共分散行列の更新
        self.x_hat += K @ (z - z_hat)
        self.P -= K @ S @ K.T

    def get_estimated_temp(self, psi_f_hat):
        """推定した磁束から温度を算出"""
        a = (85 - 25) / (0.0952 - 0.1)
        b = 25 - a * 0.1
        return a * psi_f_hat + b

# --- 3. シミュレーションの実行 (前回と同じ) ---
def main():
    SIM_TIME = 60.0
    DT = 0.0001
    LD = 0.0003
    LQ = 0.0005
    POLE_PAIRS = 4
    N_STATES = 4 # 状態変数の数

    motor = IPMSM_Simulator(DT, LD, LQ, POLE_PAIRS)
    ukf = UnscentedKalmanFilter(DT, LD, LQ, POLE_PAIRS, N_STATES)
    
    we = 200 * 2 * np.pi
    v_d_cmd = 50.0
    v_q_cmd = 100.0

    history = {
        't': [], 'i_d_true': [], 'i_q_true': [], 'i_d_est': [], 'i_q_est': [],
        'Rs_true': [], 'Rs_est': [], 'psi_f_true': [], 'psi_f_est': [],
        'temp_true': [], 'temp_est': []
    }

    for t in np.arange(0, SIM_TIME, DT):
        motor.update_true_parameters(t)
        i_d_true, i_q_true = motor.step(v_d_cmd, v_q_cmd, we)

        i_d_obs = i_d_true + np.random.normal(0, np.sqrt(ukf.R[0,0]))
        i_q_obs = i_q_true + np.random.normal(0, np.sqrt(ukf.R[1,1]))
        z = np.array([i_d_obs, i_q_obs])

        u = (v_d_cmd, v_q_cmd, we)
        ukf.predict(u)
        ukf.update(z)
        
        temp_est = ukf.get_estimated_temp(ukf.x_hat[3,0])
        
        history['t'].append(t)
        history['i_d_true'].append(i_d_true)
        history['i_q_true'].append(i_q_true)
        history['i_d_est'].append(ukf.x_hat[0,0])
        history['i_q_est'].append(ukf.x_hat[1,0])
        history['Rs_true'].append(motor.R_s_true)
        history['Rs_est'].append(ukf.x_hat[2,0])
        history['psi_f_true'].append(motor.psi_f_true)
        history['psi_f_est'].append(ukf.x_hat[3,0])
        history['temp_true'].append(motor.temp_rotor_C)
        history['temp_est'].append(temp_est)

    # --- 4. 結果のプロット (前回と同じ) ---
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('IPMSM Parameter and Temperature Estimation using UKF', fontsize=16)

    axs[0].plot(history['t'], history['Rs_true'], 'r-', label='True Rs')
    axs[0].plot(history['t'], history['Rs_est'], 'b--', label='Estimated Rs')
    axs[0].set_ylabel('Stator Resistance [ohm]')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(history['t'], history['psi_f_true'], 'r-', label='True $\Psi_f$')
    axs[1].plot(history['t'], history['psi_f_est'], 'b--', label='Estimated $\Psi_f$')
    axs[1].set_ylabel('Flux Linkage [Wb]')
    axs[1].legend()
    axs[1].grid(True)
    
    axs[2].plot(history['t'], history['temp_true'], 'r-', label='True Temperature')
    axs[2].plot(history['t'], history['temp_est'], 'b--', label='Estimated Temperature')
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('Rotor Temperature [degC]')
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    main()