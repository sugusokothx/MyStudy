import numpy as np
import matplotlib.pyplot as plt

# ヘルパー関数
def wrap_to_pi(angle):
    """角度を -pi から pi の範囲に正規化します。"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def rotate_vector(vector, angle):
    """2Dベクトルを指定された角度だけ回転させます。"""
    rotation_matrix = np.array([np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)])
    return rotation_matrix @ np.asarray(vector)

# --- SRDに基づくコンポーネントクラス ---

class FdcFilter:
    """
    直流成分除去フィルタ (SRD 4.3.1)
    SRD 式10.49a: F_dc(z^-1) = beta * (1-z^-1) / (1 + alpha1*z^-1 + alpha2*z^-2)
    差分方程式: y[k] = beta*(x[k] - x[k-1]) - alpha1*y[k-1] - alpha2*y[k-2]
    """
    def __init__(self, ts, alpha1, alpha2, beta):
        self.ts = ts
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.beta = beta
        
        # 状態変数 (gamma, delta 成分それぞれ)
        self.x_gamma_prev = 0.0
        self.y_gamma_prev1 = 0.0
        self.y_gamma_prev2 = 0.0
        
        self.x_delta_prev = 0.0
        self.y_delta_prev1 = 0.0
        self.y_delta_prev2 = 0.0

    def step(self, i_gamma, i_delta):
        # Gamma成分
        y_gamma = self.beta * (i_gamma - self.x_gamma_prev) \
                  - self.alpha1 * self.y_gamma_prev1 \
                  - self.alpha2 * self.y_gamma_prev2
        
        self.x_gamma_prev = i_gamma
        self.y_gamma_prev2 = self.y_gamma_prev1
        self.y_gamma_prev1 = y_gamma
        
        # Delta成分
        y_delta = self.beta * (i_delta - self.x_delta_prev) \
                  - self.alpha1 * self.y_delta_prev1 \
                  - self.alpha2 * self.y_delta_prev2
                  
        self.x_delta_prev = i_delta
        self.y_delta_prev2 = self.y_delta_prev1
        self.y_delta_prev1 = y_delta
        
        return y_gamma, y_delta

class ExtractingFilter:
    """
    相成分抽出フィルタ (SRD 4.3.2.1)
    LPF F(s) = a0 / (s^2 + a1*s + a0) を離散化し、周波数シフト法で正相・逆相成分を抽出。
    """
    def __init__(self, ts, a0, a1, omega_h_setting):
        self.ts = ts
        self.a0 = a0
        self.a1 = a1
        self.omega_h = omega_h_setting

        # LPF F(s) の離散化 (タスティン変換)
        # s = (2/Ts) * (1-z^-1)/(1+z^-1)
        K_tustin = 2.0 / self.ts
        
        # Denominator coefficients for F(z): 1 + a1_prime*z^-1 + a2_prime*z^-2
        den_coeff0 = K_tustin**2 + self.a1*K_tustin + self.a0
        self.lpf_a1_prime = (2*self.a0 - 2*K_tustin**2) / den_coeff0
        self.lpf_a2_prime = (K_tustin**2 - self.a1*K_tustin + self.a0) / den_coeff0
        
        # Numerator coefficients for F(z): b0_prime + b1_prime*z^-1 + b2_prime*z^-2
        self.lpf_b0_prime = self.a0 / den_coeff0
        self.lpf_b1_prime = 2*self.a0 / den_coeff0
        self.lpf_b2_prime = self.a0 / den_coeff0

        # 状態変数 (正相、逆相、それぞれ実部・虚部)
        # x_hp_real, x_hp_imag, y_hp_real, y_hp_imag (それぞれ prev1, prev2)
        self.x_hp_real_p1, self.x_hp_real_p2 = 0.0, 0.0
        self.y_hp_real_p1, self.y_hp_real_p2 = 0.0, 0.0
        self.x_hp_imag_p1, self.x_hp_imag_p2 = 0.0, 0.0
        self.y_hp_imag_p1, self.y_hp_imag_p2 = 0.0, 0.0

        # x_hn_real, x_hn_imag, y_hn_real, y_hn_imag (それぞれ prev1, prev2)
        self.x_hn_real_p1, self.x_hn_real_p2 = 0.0, 0.0
        self.y_hn_real_p1, self.y_hn_real_p2 = 0.0, 0.0
        self.x_hn_imag_p1, self.x_hn_imag_p2 = 0.0, 0.0
        self.y_hn_imag_p1, self.y_hn_imag_p2 = 0.0, 0.0

    def _apply_lpf(self, x_curr, x_prev1, x_prev2, y_prev1, y_prev2):
        y_curr = self.lpf_b0_prime * x_curr + \
                 self.lpf_b1_prime * x_prev1 + \
                 self.lpf_b2_prime * x_prev2 - \
                 self.lpf_a1_prime * y_prev1 - \
                 self.lpf_a2_prime * y_prev2
        return y_curr

    def step(self, i1h_prime_gamma, i1h_prime_delta, current_time):
        i1h_prime_complex = i1h_prime_gamma + 1j * i1h_prime_delta
        
        # 正相成分抽出 i_hp
        # 1. 入力信号を -omega_h t で変調 (ベースバンドへ)
        mod_angle_hp = -self.omega_h * current_time
        i1h_modulated_hp = i1h_prime_complex * (np.cos(mod_angle_hp) + 1j * np.sin(mod_angle_hp))
        
        # 2. LPFを実部と虚部にそれぞれ適用
        y_hp_real_curr = self._apply_lpf(i1h_modulated_hp.real, self.x_hp_real_p1, self.x_hp_real_p2, self.y_hp_real_p1, self.y_hp_real_p2)
        self.x_hp_real_p2, self.x_hp_real_p1 = self.x_hp_real_p1, i1h_modulated_hp.real
        self.y_hp_real_p2, self.y_hp_real_p1 = self.y_hp_real_p1, y_hp_real_curr
        
        y_hp_imag_curr = self._apply_lpf(i1h_modulated_hp.imag, self.x_hp_imag_p1, self.x_hp_imag_p2, self.y_hp_imag_p1, self.y_hp_imag_p2)
        self.x_hp_imag_p2, self.x_hp_imag_p1 = self.x_hp_imag_p1, i1h_modulated_hp.imag
        self.y_hp_imag_p2, self.y_hp_imag_p1 = self.y_hp_imag_p1, y_hp_imag_curr
        
        lpf_out_hp_complex = y_hp_real_curr + 1j * y_hp_imag_curr
        
        # 3. +omega_h t で復調
        demod_angle_hp = self.omega_h * current_time
        ihp_complex = lpf_out_hp_complex * (np.cos(demod_angle_hp) + 1j * np.sin(demod_angle_hp))
        ihp_gamma_delta = np.array([ihp_complex.real, ihp_complex.imag])

        # 逆相成分抽出 i_hn
        # 1. 入力信号を +omega_h t で変調
        mod_angle_hn = self.omega_h * current_time
        i1h_modulated_hn = i1h_prime_complex * (np.cos(mod_angle_hn) + 1j * np.sin(mod_angle_hn))

        # 2. LPFを実部と虚部にそれぞれ適用
        y_hn_real_curr = self._apply_lpf(i1h_modulated_hn.real, self.x_hn_real_p1, self.x_hn_real_p2, self.y_hn_real_p1, self.y_hn_real_p2)
        self.x_hn_real_p2, self.x_hn_real_p1 = self.x_hn_real_p1, i1h_modulated_hn.real
        self.y_hn_real_p2, self.y_hn_real_p1 = self.y_hn_real_p1, y_hn_real_curr

        y_hn_imag_curr = self._apply_lpf(i1h_modulated_hn.imag, self.x_hn_imag_p1, self.x_hn_imag_p2, self.y_hn_imag_p1, self.y_hn_imag_p2)
        self.x_hn_imag_p2, self.x_hn_imag_p1 = self.x_hn_imag_p1, i1h_modulated_hn.imag
        self.y_hn_imag_p2, self.y_hn_imag_p1 = self.y_hn_imag_p1, y_hn_imag_curr

        lpf_out_hn_complex = y_hn_real_curr + 1j * y_hn_imag_curr

        # 3. -omega_h t で復調
        demod_angle_hn = -self.omega_h * current_time
        ihn_complex = lpf_out_hn_complex * (np.cos(demod_angle_hn) + 1j * np.sin(demod_angle_hn))
        ihn_gamma_delta = np.array([ihn_complex.real, ihn_complex.imag])
        
        return ihp_gamma_delta, ihn_gamma_delta

class MirrorPhaseDetector:
    """鏡相検出器 (SRD 4.3.2.2)"""
    def __init__(self):
        pass

    def step(self, ihp_gamma_delta, ihn_gamma_delta):
        i_hp_gamma, i_hp_delta = ihp_gamma_delta, ihp_gamma_delta[1]
        i_hn_gamma, i_hn_delta = ihn_gamma_delta, ihn_gamma_delta[1]

        C2p = i_hp_gamma * i_hn_gamma + i_hp_delta * i_hn_delta
        S2p = i_hp_delta * i_hn_gamma - i_hp_gamma * i_hn_delta
        
        theta_re_hat = 0.5 * np.arctan2(S2p, C2p)
        return theta_re_hat

class EllipseMirrorPhaseEstimator:
    """楕円鏡相推定器 (SRD 4.3.2)"""
    def __init__(self, ts, a0_ext, a1_ext, omega_h_setting):
        self.extracting_filter = ExtractingFilter(ts, a0_ext, a1_ext, omega_h_setting)
        self.mirror_phase_detector = MirrorPhaseDetector()

    def step(self, i1h_prime_gamma, i1h_prime_delta, current_time):
        ihp_gd, ihn_gd = self.extracting_filter.step(i1h_prime_gamma, i1h_prime_delta, current_time)
        theta_re_hat = self.mirror_phase_detector.step(ihp_gd, ihn_gd)
        return theta_re_hat

class PhaseCorrection:
    """位相補正 (SRD 4.3.3)"""
    def __init__(self, Kc, K_theta):
        self.Kc = Kc
        self.K_theta = K_theta

    def step(self, theta_re_hat, i1_delta, i_delta_f_cmd):
        # SRDでは i1_delta は基本波成分抽出後の i_delta_f と仮定。
        # ここでは簡易的にモータ出力の全電流 i1_delta を使用。
        delta_theta_c = self.Kc * (i1_delta - i_delta_f_cmd)
        u_PLL = wrap_to_pi(theta_re_hat + self.K_theta * delta_theta_c) # SRDでは減算だが、式10.45aは加算
                                                                     # 図10.8では theta_re - K_theta * (-Kc (...)) = theta_re + K_theta*Kc(...)
        return u_PLL

class PhaseSynchronizerPLL:
    """位相同期器 (SRD 4.3.4)"""
    def __init__(self, ts, cn1, cn0):
        self.ts = ts
        self.cn1 = cn1 # PI比例ゲイン関連 (Kp相当)
        self.cn0 = cn0 # PI積分ゲイン関連 (Ki*Ts相当)

        self.theta_alpha_hat_prev = 0.0
        self.pi_integrator_state = 0.0
        self.e_PLL_prev = 0.0 # For derivative term if standard PI form is used. Here, SRD form.

    def step(self, u_PLL):
        e_PLL = wrap_to_pi(u_PLL - self.theta_alpha_hat_prev)
        
        # PIコントローラ C'(s) = cn1 + cn0/s
        # omega_gamma[k] = cn1 * e_PLL[k] + PI_state[k]
        # PI_state[k] = PI_state[k-1] + cn0 * e_PLL[k] * Ts
        self.pi_integrator_state += self.cn0 * e_PLL * self.ts
        omega_gamma = self.cn1 * e_PLL + self.pi_integrator_state
        
        theta_alpha_hat = wrap_to_pi(self.theta_alpha_hat_prev + omega_gamma * self.ts)
        
        self.theta_alpha_hat_prev = theta_alpha_hat
        # self.e_PLL_prev = e_PLL # if needed for other PI forms
        
        return theta_alpha_hat, omega_gamma

class LPFilterSpeed:
    """ローパスフィルタ (速度推定用) (SRD 4.3.5)"""
    def __init__(self, ts, tau_lpf=None, enabled=False):
        self.ts = ts
        self.enabled = enabled
        if self.enabled:
            if tau_lpf is None:
                raise ValueError("tau_lpf must be provided if LPFilterSpeed is enabled.")
            self.alpha = self.ts / (tau_lpf + self.ts) if (tau_lpf + self.ts) > 0 else 1.0
            self.y_prev = 0.0
        
    def step(self, omega_gamma_in):
        if not self.enabled:
            return omega_gamma_in
        
        y_k = (1.0 - self.alpha) * self.y_prev + self.alpha * omega_gamma_in
        self.y_prev = y_k
        return y_k

class HFVC:
    """高周波電圧指令器 (SRD 4.3.6)"""
    def __init__(self, omega_h_setting, V_h_setting, mode="constant_amplitude"):
        self.omega_h = omega_h_setting
        self.V_h = V_h_setting
        self.mode = mode # "constant_amplitude" or "speed_adaptive"

    def step(self, current_time, omega_2n_hat_in=0.0):
        if self.mode == "constant_amplitude":
            amplitude = self.V_h
        elif self.mode == "speed_adaptive":
            if self.omega_h == 0: # Avoid division by zero
                 amplitude = self.V_h 
            else:
                 amplitude = self.V_h * (1.0 + omega_2n_hat_in / self.omega_h)
        else:
            raise ValueError("Invalid HFVC mode")

        v_gamma_h_cmd = amplitude * np.cos(self.omega_h * current_time)
        v_delta_h_cmd = amplitude * np.sin(self.omega_h * current_time)
        
        return np.array([v_gamma_h_cmd, v_delta_h_cmd])

# --- 簡易モータモデル ---
class SimpleMotorModel:
    """簡易的なモータ電流模擬モデル (オイラー法)"""
    def __init__(self, ts, Lh, Rh): # 高周波インピーダンスパラメータ
        self.ts = ts
        self.Lh = Lh if Lh > 0 else 1e-6 # Avoid division by zero
        self.Rh = Rh
        self.i_gamma_h_prev = 0.0
        self.i_delta_h_prev = 0.0

    def step(self, v_gamma_h_cmd, v_delta_h_cmd, i_gamma_f=0.0, i_delta_f=0.0):
        # オイラー法で高周波電流を計算
        # v_cmd is considered as v_cmd[k]
        # i_h[k] = i_h[k-1] + (Ts/Lh) * (v_cmd[k] - Rh * i_h[k-1])
        i_gamma_h = self.i_gamma_h_prev + (self.ts / self.Lh) * (v_gamma_h_cmd - self.Rh * self.i_gamma_h_prev)
        i_delta_h = self.i_delta_h_prev + (self.ts / self.Lh) * (v_delta_h_cmd - self.Rh * self.i_delta_h_prev)

        self.i_gamma_h_prev = i_gamma_h
        self.i_delta_h_prev = i_delta_h

        # 全電流 = 基本波電流 + 高周波電流
        i_gamma = i_gamma_f + i_gamma_h
        i_delta = i_delta_f + i_delta_h
        return np.array([i_gamma, i_delta])

# --- シミュレーション設定 ---
if __name__ == "__main__":
    # シミュレーションパラメータ (SRD 表SRD-1 および資料[1] P26-27 より)
    Ts = 125e-6  # サンプリング時間 (s)
    total_time = 0.1  # シミュレーション総時間 (s)
    
    # FdcFilter パラメータ (ユーザーが設計または指定)
    # これらは例であり、実際の設計が必要です。
    # 例えば、fc=50Hzの1次HPFを双一次変換し、SRDの2次構造に合わせるか、
    # SRDの構造に合うように2次HPFを設計します。
    # ここでは仮の値を設定します。
    alpha1_fdc = -1.9 # 仮の値
    alpha2_fdc = 0.95 # 仮の値
    beta_fdc = 0.1    # 仮の値 (ωhでゲイン1になるように調整が必要)

    # ExtractingFilter パラメータ
    a0_ext = 62500.0
    a1_ext = 500.0
    
    # PhaseCorrection パラメータ
    Kc_pc = 0.026
    K_theta_pc = 1.0
    
    # PhaseSynchronizerPLL パラメータ
    cn1_pll = 150.0
    cn0_pll = 5625.0
    
    # LPFilterSpeed パラメータ (実験では省略されたため、enabled=False)
    tau_lpf_speed = 0.01 # 例: 10ms (enabled=Trueの場合)
    lpf_speed_enabled = False # 実験設定に合わせる
    
    # HFVC パラメータ
    omega_h_hfvc = 2 * np.pi * 400  # rad/s
    V_h_hfvc = 23.0  # V
    hfvc_mode = "constant_amplitude" # 実験設定に合わせる
    
    # SimpleMotorModel パラメータ (高周波等価インピーダンス) - 仮の値
    Lh_motor = 0.5e-3 # 0.5 mH
    Rh_motor = 0.1    # 0.1 Ohm

    # 外部指令値
    i_delta_f_cmd_sim = 0.0 # 位相補正用指令値 (A) - 例として0A
    
    # 基本波電流 (モータモデル用) - 簡単のためゼロとする
    i_gamma_f_motor = 0.0
    i_delta_f_motor = 0.0 # 位相補正で使われる i_delta_f とは別。これはモータに流れる実際の基本波。
                          # SRDの位相補正入力 i1_delta はモータの全電流のδ成分を使う。

    # --- コンポーネントのインスタンス化 ---
    fdc_filter = FdcFilter(Ts, alpha1_fdc, alpha2_fdc, beta_fdc)
    ellipse_estimator = EllipseMirrorPhaseEstimator(Ts, a0_ext, a1_ext, omega_h_hfvc)
    phase_corrector = PhaseCorrection(Kc_pc, K_theta_pc)
    pll = PhaseSynchronizerPLL(Ts, cn1_pll, cn0_pll)
    speed_lpf = LPFilterSpeed(Ts, tau_lpf_speed, enabled=lpf_speed_enabled)
    hfvc_gen = HFVC(omega_h_hfvc, V_h_hfvc, mode=hfvc_mode)
    motor = SimpleMotorModel(Ts, Lh_motor, Rh_motor)

    # --- シミュレーションループ ---
    num_steps = int(total_time / Ts)
    time_vec = np.linspace(0, total_time, num_steps)
    
    # 結果保存用配列
    theta_alpha_hat_hist = np.zeros(num_steps)
    omega_2n_hat_hist = np.zeros(num_steps)
    v_gamma_h_cmd_hist = np.zeros(num_steps)
    v_delta_h_cmd_hist = np.zeros(num_steps)
    i_gamma_hist = np.zeros(num_steps)
    i_delta_hist = np.zeros(num_steps)
    theta_re_hat_hist = np.zeros(num_steps)
    u_pll_hist = np.zeros(num_steps)
    omega_gamma_hist = np.zeros(num_steps)

    # 初期値
    omega_2n_hat_current = 0.0

    for k in range(num_steps):
        current_t = time_vec[k]
        
        # 1. HFVC: 高周波電圧指令を生成
        v_h_cmd = hfvc_gen.step(current_t, omega_2n_hat_current)
        v_gamma_h_cmd_hist[k] = v_h_cmd
        v_delta_h_cmd_hist[k] = v_h_cmd[1]
        
        # 2. モータモデル: 電流を模擬
        #    基本波電流 i_gamma_f_motor, i_delta_f_motor はここでは0と仮定
        i1_current = motor.step(v_h_cmd, v_h_cmd[1], i_gamma_f_motor, i_delta_f_motor)
        i_gamma_hist[k] = i1_current
        i_delta_hist[k] = i1_current[1]
        
        # 3. FdcFilter: 直流成分除去
        i1h_prime_gamma, i1h_prime_delta = fdc_filter.step(i1_current, i1_current[1])
        
        # 4. EllipseMirrorPhaseEstimator: 回転子長軸位相を推定
        theta_re_hat_current = ellipse_estimator.step(i1h_prime_gamma, i1h_prime_delta, current_t)
        theta_re_hat_hist[k] = theta_re_hat_current
        
        # 5. PhaseCorrection: 位相補正
        #    i1_delta はモータ出力の全電流のδ成分 i1_current[1] を使用
        u_PLL_current = phase_corrector.step(theta_re_hat_current, i1_current[1], i_delta_f_cmd_sim)
        u_pll_hist[k] = u_PLL_current
        
        # 6. PhaseSynchronizerPLL: 位相と速度を同期
        theta_alpha_hat_current, omega_gamma_current = pll.step(u_PLL_current)
        theta_alpha_hat_hist[k] = theta_alpha_hat_current
        omega_gamma_hist[k] = omega_gamma_current
        
        # 7. LPFilterSpeed: 速度をフィルタリング
        omega_2n_hat_current = speed_lpf.step(omega_gamma_current)
        omega_2n_hat_hist[k] = omega_2n_hat_current

    # --- 結果のプロット ---
    plt.figure(figsize=(12, 10))
    
    plt.subplot(4, 2, 1)
    plt.plot(time_vec, theta_alpha_hat_hist)
    plt.title('Estimated Rotor Phase (theta_alpha_hat)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid(True)

    plt.subplot(4, 2, 2)
    plt.plot(time_vec, omega_2n_hat_hist)
    plt.title('Estimated Rotor Speed (omega_2n_hat)')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (rad/s)')
    plt.grid(True)

    plt.subplot(4, 2, 3)
    plt.plot(time_vec, v_gamma_h_cmd_hist, label='V_gamma_h_cmd')
    plt.plot(time_vec, v_delta_h_cmd_hist, label='V_delta_h_cmd')
    plt.title('HF Voltage Command (v1h_cmd)')
    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 2, 4)
    plt.plot(time_vec, i_gamma_hist, label='i_gamma')
    plt.plot(time_vec, i_delta_hist, label='i_delta')
    plt.title('Stator Current (i1)')
    plt.xlabel('Time (s)')
    plt.ylabel('Current (A)')
    plt.legend()
    plt.grid(True)

    plt.subplot(4, 2, 5)
    plt.plot(time_vec, theta_re_hat_hist)
    plt.title('Estimated Ellipse Major Axis Phase (theta_re_hat)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid(True)

    plt.subplot(4, 2, 6)
    plt.plot(time_vec, u_pll_hist)
    plt.title('PLL Input (u_PLL)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid(True)
    
    plt.subplot(4, 2, 7)
    plt.plot(time_vec, omega_gamma_hist)
    plt.title('PLL Output Speed (omega_gamma)')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (rad/s)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()