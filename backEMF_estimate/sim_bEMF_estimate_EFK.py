
#EKF
import numpy as np
import matplotlib.pyplot as plt

# --- 1. IPMSM（シミュレータ）クラスの定義 ---
class IPMSM_Simulator:
    """
    IPMモータの実際の動作をシミュレートするクラス。
    温度上昇によるパラメータ変動を再現する。
    """
    def __init__(self, dt, Ld, Lq, pole_pairs):
        # モータの基本パラメータ
        self.dt = dt
        self.Ld = Ld
        self.Lq = Lq
        self.pole_pairs = pole_pairs

        # 初期状態（真値）
        self.temp_rotor_C = 25.0  # ロータ温度 [℃]
        self.R_s_true = 0.05  # 固定子抵抗 [ohm]
        self.psi_f_true = 0.1   # 永久磁石鎖交磁束 [Wb]
        self.i_d = 0.0
        self.i_q = 0.0

        # 温度依存性の係数（仮の値）
        self.alpha_R = 0.00393 # 抵抗の温度係数
        self.alpha_psi = -0.0008 # 磁束の温度係数

    def update_true_parameters(self, t):
        """時間経過とともに温度とパラメータを変化させる"""
        # 簡単な温度上昇モデル（例：60秒で60℃上昇）
        if self.temp_rotor_C < 85.0:
            self.temp_rotor_C = 25.0 + t
        
        # 温度に基づき、抵抗と磁束の真値を更新
        self.R_s_true = 0.05 * (1 + self.alpha_R * (self.temp_rotor_C - 25.0))
        self.psi_f_true = 0.1 * (1 + self.alpha_psi * (self.temp_rotor_C - 25.0))

    def step(self, v_d, v_q, we):
        """1ステップシミュレーションを進める"""
        # モータの電圧方程式 (オイラー法で離散化)
        di_d_dt = (1/self.Ld) * (-self.R_s_true * self.i_d + we * self.Lq * self.i_q + v_d)
        di_q_dt = (1/self.Lq) * (-self.R_s_true * self.i_q - we * self.Ld * self.i_d - we * self.psi_f_true + v_q)
        
        self.i_d += di_d_dt * self.dt
        self.i_q += di_q_dt * self.dt
        
        return self.i_d, self.i_q

# --- 2. 拡張カルマンフィルタ（EKF）クラスの定義 ---
class ExtendedKalmanFilter:
    """
    固定子抵抗と磁束を推定するためのEKF
    状態ベクトル x = [i_d, i_q, R_s, psi_f]^T
    """
    def __init__(self, dt, Ld, Lq, pole_pairs):
        self.dt = dt
        self.Ld = Ld
        self.Lq = Lq
        self.pole_pairs = pole_pairs
        
        # 状態ベクトル x = [i_d, i_q, R_s, psi_f]^T の初期推定値
        self.x_hat = np.array([0.0, 0.0, 0.04, 0.11]).reshape(4, 1)

        # 誤差共分散行列 P の初期値
        self.P = np.diag([1e-3, 1e-3, 1e-4, 1e-4])

        # プロセスノイズ共分散行列 Q
        self.Q = np.diag([1e-5, 1e-5, 1e-9, 1e-10])

        # 観測ノイズ共分散行列 R
        self.R = np.diag([1e-4, 1e-4])
        
        # 観測行列 H
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    def predict(self, u):
        """予測ステップ"""
        v_d, v_q, we = u
        i_d, i_q, R_s, psi_f = self.x_hat.flatten()

        # --- 状態方程式 f(x, u) ---
        x_dot = np.array([
            (1/self.Ld) * (-R_s * i_d + we * self.Lq * i_q + v_d),
            (1/self.Lq) * (-R_s * i_q - we * self.Ld * i_d - we * psi_f + v_q),
            0, # 抵抗はランダムウォークモデル
            0  # 磁束はランダムウォークモデル
        ]).reshape(4, 1)
        
        # --- 状態遷移行列 F (f(x,u)のヤコビアン) ---
        F = np.zeros((4, 4))
        F[0, 0] = -R_s / self.Ld
        F[0, 1] = we * self.Lq / self.Ld
        F[0, 2] = -i_d / self.Ld
        
        F[1, 0] = -we * self.Ld / self.Lq
        F[1, 1] = -R_s / self.Lq
        F[1, 2] = -i_q / self.Lq
        F[1, 3] = -we / self.Lq
        # R_s, psi_f のダイナミクスはゼロなので、3,4行目はゼロ
        
        # 離散化 (オイラー法)
        F_k = np.eye(4) + F * self.dt
        self.x_hat = self.x_hat + x_dot * self.dt
        
        # 誤差共分散行列の予測
        self.P = F_k @ self.P @ F_k.T + self.Q

    def update(self, z):
        """更新ステップ"""
        # z: 観測ベクトル [i_d_obs, i_q_obs]^T
        z = z.reshape(2, 1)

        # カルマンゲインの計算
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 状態ベクトルと誤差共分散行列の更新
        y = z - self.H @ self.x_hat
        self.x_hat = self.x_hat + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
    def get_estimated_temp(self, psi_f_hat):
        """推定した磁束から温度を算出（論文の線形関係を仮定）"""
        # T_rotor = a * psi_f + b
        # この係数 a, b は事前に実験で求めておく必要がある
        # 例：T(psi=0.1)=25℃, T(psi=0.0952)=85℃ と仮定
        # 0.0952 = 0.1 * (1 + (-0.0008) * (85-25))
        a = (85 - 25) / (0.0952 - 0.1)
        b = 25 - a * 0.1
        return a * psi_f_hat + b

# --- 3. シミュレーションの実行 ---
def main():
    # シミュレーション設定
    SIM_TIME = 60.0  # [s]
    DT = 0.0001      # [s]
    
    # モータパラメータ
    LD = 0.0003  # d軸インダクタンス [H]
    LQ = 0.0005  # q軸インダクタンス [H]
    POLE_PAIRS = 4

    # インスタンス生成
    motor = IPMSM_Simulator(DT, LD, LQ, POLE_PAIRS)
    ekf = ExtendedKalmanFilter(DT, LD, LQ, POLE_PAIRS)
    
    # 制御指令値（例：定トルク運転）
    we = 200 * 2 * np.pi  # 電気角周波数 200Hz [rad/s]
    v_d_cmd = 50.0   # d軸電圧指令 [V]
    v_q_cmd = 100.0  # q軸電圧指令 [V]

    # データ記録用リスト
    history = {
        't': [], 'i_d_true': [], 'i_q_true': [], 'i_d_est': [], 'i_q_est': [],
        'Rs_true': [], 'Rs_est': [], 'psi_f_true': [], 'psi_f_est': [],
        'temp_true': [], 'temp_est': []
    }

    # シミュレーションループ
    for t in np.arange(0, SIM_TIME, DT):
        # 1. モータの真の状態を更新
        motor.update_true_parameters(t)
        i_d_true, i_q_true = motor.step(v_d_cmd, v_q_cmd, we)

        # 2. 観測データを作成（センサノイズを付加）
        i_d_obs = i_d_true + np.random.normal(0, np.sqrt(ekf.R[0,0]))
        i_q_obs = i_q_true + np.random.normal(0, np.sqrt(ekf.R[1,1]))
        z = np.array([i_d_obs, i_q_obs])

        # 3. EKFを実行
        u = (v_d_cmd, v_q_cmd, we)
        ekf.predict(u)
        ekf.update(z)
        
        # 4. 推定された磁束から温度を計算
        temp_est = ekf.get_estimated_temp(ekf.x_hat[3,0])
        
        # 5. データを記録
        history['t'].append(t)
        history['i_d_true'].append(i_d_true)
        history['i_q_true'].append(i_q_true)
        history['i_d_est'].append(ekf.x_hat[0,0])
        history['i_q_est'].append(ekf.x_hat[1,0])
        history['Rs_true'].append(motor.R_s_true)
        history['Rs_est'].append(ekf.x_hat[2,0])
        history['psi_f_true'].append(motor.psi_f_true)
        history['psi_f_est'].append(ekf.x_hat[3,0])
        history['temp_true'].append(motor.temp_rotor_C)
        history['temp_est'].append(temp_est)

    # --- 4. 結果のプロット ---
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('IPMSM Parameter and Temperature Estimation using EKF', fontsize=16)

    # 固定子抵抗の推定結果
    axs[0].plot(history['t'], history['Rs_true'], 'r-', label='True Rs')
    axs[0].plot(history['t'], history['Rs_est'], 'b--', label='Estimated Rs')
    axs[0].set_ylabel('Stator Resistance [ohm]')
    axs[0].legend()
    axs[0].grid(True)

    # 磁石鎖交磁束の推定結果
    axs[1].plot(history['t'], history['psi_f_true'], 'r-', label='True $\Psi_f$')
    axs[1].plot(history['t'], history['psi_f_est'], 'b--', label='Estimated $\Psi_f$')
    axs[1].set_ylabel('Flux Linkage [Wb]')
    axs[1].legend()
    axs[1].grid(True)
    
    # ロータ温度の推定結果
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