import numpy as np
from scipy.signal import butter, lfilter, group_delay

class VRAFPhaseComp:
    """
    Vector-Rotator-Assisted Filter with phase-delay compensation.

    Parameters
    ----------
    f_shift : float   – 抽出したい周波数 [Hz]
    Fs      : float   – サンプリング周波数 [Hz]
    seq     : str     – 'pos' (＋) or 'neg' (−) sequence
    fc      : float   – LPF カットオフ [Hz]
    order   : int     – Butterworth 次数（偶数）
    gd_manual : float | None
        LPF の遅延 [samples] を外部指定したいときに使う。
        None の場合は scipy.signal.group_delay で自動推定。
    """
    def __init__(self, f_shift, Fs, seq='pos',
                 fc=400, order=4, gd_manual=None):
        self.ω = 2*np.pi * f_shift
        self.Fs = Fs
        self.seq = seq
        self.t_idx = 0

        # --- LPF 設計 ---
        self.b, self.a = butter(order, fc/(Fs*0.5), btype='low')

        # --- group delay 推定 [samples] ---
        if gd_manual is None:
            w, gd = group_delay((self.b, self.a), w=[0])  # w=0 だけ見る
            self.gd_samp = float(gd[0])                  # 実数 (≒整数)
        else:
            self.gd_samp = float(gd_manual)
        self.gd_sec = self.gd_samp / Fs                 # [s]

        # 状態ベクトル
        n_state = max(len(self.a), len(self.b)) - 1
        self.zR = np.zeros(n_state)
        self.zI = np.zeros_like(self.zR)

    # -----------------------------------
    def reset(self):
        self.t_idx = 0
        self.zR[:] = 0
        self.zI[:] = 0

    # -----------------------------------
    def process(self, u_alpha, u_beta):
        """
        1 サンプル処理 → 位相補償済み (α, β) を返す
        """
        # --- 入力を複素数化 ---
        u_p = u_alpha + 1j*u_beta
        t = self.t_idx / self.Fs
        self.t_idx += 1

        # --- デモジュ ---
        if   self.seq == 'pos':
            u_tilde  = u_p * np.exp(-1j * self.ω * t)
            # このあと使う位相補償付きリモジュ係数
            rot_back = np.exp(+1j * self.ω * (t - self.gd_sec))
        else:  # negative-sequence
            u_tilde  = u_p * np.exp(+1j * self.ω * t)
            rot_back = np.exp(-1j * self.ω * (t - self.gd_sec))

        # --- LPF（実数部・虚数部それぞれ） ---
        xR, self.zR = lfilter(self.b, self.a, [u_tilde.real], zi=self.zR)
        xI, self.zI = lfilter(self.b, self.a, [u_tilde.imag], zi=self.zI)
        x_tilde = xR[0] + 1j*xI[0]

        # --- 位相補償付きリモジュ ---
        y_p = x_tilde * rot_back
        return y_p.real, y_p.imag
