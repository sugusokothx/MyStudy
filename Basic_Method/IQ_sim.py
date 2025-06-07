import numpy as np

class IQDemodulator:
    """
    高周波電流 Ihα, Ihβ を I/Q 同期検波で抽出するユーティリティ

    Parameters
    ----------
    omega_h : float
        注入電圧の角周波数 [rad/s]
    Ts : float
        サンプリング周期 [s]
    tau_lpf : float
        復調後ベースバンド信号を平滑化する一次 LPF の時定数 [s]
    mode : {'sin', 'cos', 'complex'}
        • 'sin'  : sin(ωt) で復調（従来の片側分解）  
        • 'cos'  : cos(ωt) で復調  
        • 'complex' : sin + j cos の複素 IQ で復調
    """

    def __init__(self,
                 omega_h: float,
                 Ts: float,
                 tau_lpf: float = 1e-4,
                 mode: str = "complex") -> None:

        self.ωh = omega_h
        self.Ts = Ts
        self._alpha = Ts / (tau_lpf + Ts)     # 1st-order IIR 係数
        self.mode = mode

        # LPF 内部状態（sin / cos の各枝 × αβ で計 4 つ）
        self._lpf_s_α = self._lpf_s_β = 0.0
        self._lpf_c_α = self._lpf_c_β = 0.0

    # ──────────────────────────────────────────
    # 内部: 1st-order IIR LPF
    # ──────────────────────────────────────────
    def _lpf(self, prev: float, new: float) -> float:
        """一次 IIR : y_k = y_{k-1} + α (x_k − y_{k-1})"""
        return prev + self._alpha * (new - prev)

    # ──────────────────────────────────────────
    # 公開 API
    # ──────────────────────────────────────────
    def update(self, Iα: float, Iβ: float, t: float):
        """
        αβ 電流 1 サンプルを入力し、復調後の HF 成分を返す

        Returns
        -------
        (Ihα, Ihβ)            : mode='sin' または 'cos'
        Ih_complex (α+jβ)      : mode='complex'
        """
        sin_ref = np.sin(self.ωh * t)
        cos_ref = np.cos(self.ωh * t)

        # ── sin 枝 ──
        self._lpf_s_α = self._lpf(self._lpf_s_α, Iα * sin_ref)
        self._lpf_s_β = self._lpf(self._lpf_s_β, Iβ * sin_ref)

        # ── cos 枝 ──
        self._lpf_c_α = self._lpf(self._lpf_c_α, Iα * cos_ref)
        self._lpf_c_β = self._lpf(self._lpf_c_β, Iβ * cos_ref)

        # sin², cos² の平均 0.5 を補正 → 振幅 ×2
        if self.mode == "sin":
            return 2 * self._lpf_s_α, 2 * self._lpf_s_β

        if self.mode == "cos":
            return 2 * self._lpf_c_α, 2 * self._lpf_c_β

        if self.mode == "complex":
            Ihα = 2 * (self._lpf_s_α + 1j * self._lpf_c_α)
            Ihβ = 2 * (self._lpf_s_β + 1j * self._lpf_c_β)
            return Ihα + 1j * Ihβ

        raise ValueError("mode must be 'sin', 'cos', or 'complex'")

# ──────────────────── テスト例 ────────────────────
if __name__ == "__main__":
    f_h  = 1_000.0
    ωh   = 2*np.pi*f_h
    Ts   = 2e-6
    Tsim = 2e-3

    demod = IQDemodulator(ωh, Ts, tau_lpf=1e-4, mode="complex")

    for k in range(int(Tsim / Ts)):
        t  = k * Ts
        Iα = 0.4 * np.sin(ωh*t + 0.3)
        Iβ = 0.4 * np.cos(ωh*t + 0.3)
        Ih = demod.update(Iα, Iβ, t)
        # print(Ih)  # テキスト出力や可視化はお好みで
