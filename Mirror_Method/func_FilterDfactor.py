import numpy as np
import matplotlib.pyplot as plt

# 以前のコードと同じ DFactorFilter クラスをここに配置する
class DFactorFilter:
    """
    資料の9.3節で解説されている二相入力・二相出力のD因子フィルタを実装したクラス。
    """
    def __init__(self, a0, b0, b1, omega0, fs):
        self.a0 = a0
        self.b0 = b0
        self.b1 = b1
        self.omega0 = omega0
        self.Ts = 1.0 / fs
        self.x_p = 0 + 0j
        
    def process(self, u):
        u_p = u[0] + 1j * u[1]
        x_p_new = (self.x_p + self.Ts * u_p) / (1 + (self.a0 + 1j * self.omega0) * self.Ts)
        y_p = (self.b0 - self.a0 * self.b1) * x_p_new + self.b1 * u_p
        self.x_p = x_p_new
        return np.array([y_p.real, y_p.imag])

# --- メイン実行部 ---
if __name__ == '__main__':
    # シミュレーション設定
    FS = 12000  # サンプリング周波数: 12kHz
    T = 0.05    # シミュレーション時間 (s)
    t = np.arange(0, T, 1.0/FS)

    # フィルタのパラメータ設定
    # F(s) = 100/(s+100) -> a0=100, b0=100, b1=0
    # このフィルタは帯域幅が約100 rad/s (約16Hz)のローパスフィルタ
    A0 = 100.0
    B0 = 100.0
    B1 = 0.0
    
    # シフト周波数: -1kHz -> -2*pi*1000 rad/s
    # (通過帯域の中心が 1kHz になるように負値を設定)
    OMEGA0_SHIFT = -2 * np.pi * 1000.0

    # D因子フィルタのインスタンスを作成
    d_filter = DFactorFilter(a0=A0, b0=B0, b1=B1, omega0=OMEGA0_SHIFT, fs=FS)

    # テスト入力信号の生成 (1kHz信号 + 3kHzノイズ)
    freq_target = 1000  # Hz (フィルタで抽出したい周波数)
    w_target = 2 * np.pi * freq_target
    signal = np.array([np.cos(w_target * t), np.sin(w_target * t)])

    freq_noise = 3000   # Hz (除去したいノイズ周波数)
    w_noise = 2 * np.pi * freq_noise
    noise = np.array([np.cos(w_noise * t), np.sin(w_noise * t)]) * 0.5
    
    input_signal = (signal + noise).T

    # フィルタ処理の実行
    output_signal = np.zeros_like(input_signal)
    for i in range(len(t)):
        output_signal[i, :] = d_filter.process(input_signal[i, :])

    # 結果のプロット
    plt.figure(figsize=(12, 6))
    plt.plot(t, input_signal[:, 0], 'c-', label='Input u1 (1kHz + 3kHz Noise)', alpha=0.7)
    plt.plot(t, output_signal[:, 0], 'b-', label='Output y1 (1kHz Filtered)')
    plt.title("D-Factor Filter Response (Fs=12kHz, F_shift=1kHz)")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.xlim(0, 0.01) # 波形が見やすいように時間範囲を拡大
    plt.grid(True)
    plt.legend()
    plt.show()