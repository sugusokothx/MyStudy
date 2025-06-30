# ───────────────────────────────────────────────
# Vector-Rotator-Assisted Filter (VRAF) – demo script
#  - Handles both positive-sequence (+) and negative-sequence (−) harmonics
#  - Example: fundamental 200 Hz (+) + harmonic 1 kHz (sign selectable)
#    Fs = 12 kHz, duration = 50 ms
# ───────────────────────────────────────────────
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter

# -----------------------------
# 1)  Core class
# -----------------------------
import numpy as np
from scipy.signal import butter, lfilter

class VRAFReal:
    """
    Vector-Rotator-Assisted Filter (real-only version).
    Completely avoids complex numbers while keeping identical performance.
    """
    def __init__(self, f_shift, Fs, seq='pos', fc=150, order=4):
        self.Fs = Fs
        self.seq = seq  # 'pos' or 'neg'
        self.omega = 2*np.pi*f_shift
        self.delta = self.omega / Fs          # phase increment per sample
        self.cosθ, self.sinθ = 1.0, 0.0       # current cosθ_n, sinθ_n

        # Butterworth LPF (real coefficients)
        self.b, self.a = butter(order, fc/(Fs*0.5), btype='low')
        n_state = max(len(self.a), len(self.b)) - 1
        self.zR = np.zeros(n_state)           # LPF state for α-like component
        self.zI = np.zeros(n_state)           # LPF state for β-like component

        # lpf = ButterworthLPF4(fc=150, fs=12_000)
        assert order == 4, "VRAFReal+ButterworthLPF4 は 4-th 専用"
        self.lpf_R = ButterworthLPF4(fc, Fs)   # uR (実) 用
        self.lpf_I = ButterworthLPF4(fc, Fs)   # uI (虚) 用

    # ---------- low-level helpers ----------
    def _rot_fwd(self, u_alpha, u_beta):
        """Rotate [uα,uβ] by ±θ_n for demodulation."""
        if self.seq == 'pos':    # use –θ  (extract positive-seq)
            uR =  u_alpha * self.cosθ +  u_beta * self.sinθ
            uI = -u_alpha * self.sinθ +  u_beta * self.cosθ
        else:                    # use +θ  (extract negative-seq)
            uR =  u_alpha * self.cosθ -  u_beta * self.sinθ
            uI =  u_alpha * self.sinθ +  u_beta * self.cosθ
        return uR, uI

    def _rot_back(self, xR, xI):
        """Rotate filtered pair back to original frame."""
        if self.seq == 'pos':    # use +θ
            y_alpha = xR * self.cosθ - xI * self.sinθ
            y_beta  = xR * self.sinθ + xI * self.cosθ
        else:                    # use –θ
            y_alpha = xR * self.cosθ + xI * self.sinθ
            y_beta  = -xR * self.sinθ + xI * self.cosθ
        return y_alpha, y_beta

    def _advance_phase(self):
        """Update cosθ, sinθ using a recurrence (avoids expensive sin/cos calls)."""
        cosΔ, sinΔ = np.cos(self.delta), np.sin(self.delta)
        c_new = self.cosθ * cosΔ - self.sinθ * sinΔ
        s_new = self.sinθ * cosΔ + self.cosθ * sinΔ
        self.cosθ, self.sinθ = c_new, s_new

    # ---------- public API ----------
    def reset(self):
        self.cosθ, self.sinθ = 1.0, 0.0
        self.zR[:] = 0.0
        self.zI[:] = 0.0
        self.lpf_R.reset()
        self.lpf_I.reset()

    def process(self, u_alpha, u_beta):
        """Process one αβ sample and return the extracted component (α,β)."""
        # 1. demodulate (rotate into dc)
        uR, uI = self._rot_fwd(u_alpha, u_beta)

        # 2. low-pass real & imag parts separately
        # xR, self.zR = lfilter(self.b, self.a, [uR], zi=self.zR)
        # xI, self.zI = lfilter(self.b, self.a, [uI], zi=self.zI)
        xR = self.lpf_R.process_sample(uR)
        xI = self.lpf_I.process_sample(uI)
        # 3. remodulate (rotate back)

        # y_alpha, y_beta = self._rot_back(xR[0], xI[0])
        y_alpha, y_beta = self._rot_back(xR, xI)

        # 4. advance phase for next sample
        self._advance_phase()
        return y_alpha, y_beta

class ButterworthLPF4:
    """Real‑time 4‑th‑order Butterworth low‑pass filter (LPF).

    Implemented as a **cascade of two Direct‑Form‑II transposed biquads**
    for numerical robustness.  The coefficient generator is the
    `design_butter4` function above.
    """

    def __init__(self, fc: float, fs: float):
        # Store second‑order sections (b0,b1,b2,a1,a2)
        sos = []
        omega_c = 2.0 * fs * tan(pi * fc / fs)
        for ζ in [sin(3.0 * pi / 8.0), sin(5.0 * pi / 8.0)]:
            b, a = _biquad_coeff(omega_c, fs, ζ)
            sos.append((b[0], b[1], b[2], a[1], a[2]))
        self.sos = sos  # two tuples
        self.z = np.zeros((2, 2), dtype=float)  # [section][state]

    # ---------------------------------------------------------
    def reset(self):
        self.z[:, :] = 0.0

    def process_sample(self, x: float) -> float:
        """Filter a single sample (float in, float out)."""
        for i, (b0, b1, b2, a1, a2) in enumerate(self.sos):
            # direct‑form‑II transposed biquad
            y = b0 * x + self.z[i, 0]
            self.z[i, 0] = b1 * x + self.z[i, 1] - a1 * y
            self.z[i, 1] = b2 * x - a2 * y
            x = y  # output of this stage is input to next
        return x

    def process_array(self, x: np.ndarray) -> np.ndarray:
        """Vectorised filtering for convenience (uses process_sample)."""
        y = np.empty_like(x, dtype=float)
        for n, val in enumerate(x):
            y[n] = self.process_sample(float(val))
        return y


class VRAF:
    """
    Vector-Rotator-Assisted Filter for αβ two-phase signals.

    Parameters
    ----------
    f_shift : float
        Target harmonic frequency [Hz] to extract (|f|).
    Fs : float
        Sampling frequency [Hz].
    seq : str
        'pos'  → positive-sequence (+) component
        'neg'  → negative-sequence (−) component
    fc : float
        IIR low-pass cutoff frequency [Hz].
    order : int
        IIR Butterworth order (even).
    """
    def __init__(self, f_shift, Fs, seq='pos', fc=150, order=4):
        self.ω = 2*np.pi*f_shift
        self.Fs = Fs
        self.seq = seq
        self.t_idx = 0
        # Butterworth LPF design
        self.b, self.a = butter(order, fc/(Fs*0.5), btype='low')
        self.zR = np.zeros(max(len(self.a), len(self.b))-1)  # lfilter state
        self.zI = np.zeros_like(self.zR)

    def reset(self):
        self.t_idx = 0
        self.zR[:] = 0
        self.zI[:] = 0

    def process(self, u_alpha, u_beta):
        """Process single sample, return filtered α,β."""
        # complex input
        u_p = u_alpha + 1j*u_beta
        t = self.t_idx / self.Fs
        self.t_idx += 1
        # choose rotation sign
        if self.seq == 'pos':      # extract positive-sequence
            u_tilde = u_p * np.exp(-1j*self.ω*t)   # demod
            rot_back = np.exp(+1j*self.ω*t)
        else:                      # extract negative-sequence
            u_tilde = u_p * np.exp(+1j*self.ω*t)
            rot_back = np.exp(-1j*self.ω*t)
        # IIR LPF – run real & imag separately
        xR, self.zR = lfilter(self.b, self.a, [u_tilde.real], zi=self.zR)
        xI, self.zI = lfilter(self.b, self.a, [u_tilde.imag], zi=self.zI)
        x_tilde = xR[0] + 1j*xI[0]
        # remodulate
        y_p = x_tilde * rot_back
        return y_p.real, y_p.imag


# -----------------------------
# 2)  Simulation settings
# -----------------------------
Fs   = 12_000            # [Hz]
Ts   = 1 / Fs
Tend = 0.05              # [s]
t = np.arange(0, Tend, Ts)

f_base = 100             # [Hz] fundamental (positive-sequence)
f_harm = 1000            # [Hz] harmonic
harm_seq = 'neg'         # 'pos' or 'neg'

# build input αβ
Va_1 = 4
u_alpha = Va_1*np.cos(2*np.pi*f_base*t) + np.cos(2*np.pi*f_harm*t)
u_beta  = Va_1*np.sin(2*np.pi*f_base*t)
if harm_seq == 'pos':
    u_beta += np.sin(2*np.pi*f_harm*t)
else:  # negative-sequence: minus sign on β
    u_beta -= np.sin(2*np.pi*f_harm*t)

# -----------------------------
# 3)  Run VRAF
# -----------------------------
vraf = VRAFReal(f_shift=f_harm, Fs=Fs, seq=harm_seq, fc=150, order=4)

y_alpha = np.zeros_like(u_alpha)
y_beta  = np.zeros_like(u_beta)

for k in range(len(t)):
    y_alpha[k], y_beta[k] = vraf.process(u_alpha[k], u_beta[k])

# -----------------------------
# 4)  Plot results
# -----------------------------
fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
ax[0].plot(t, u_alpha, label='input $u_\\alpha$')
ax[0].plot(t, y_alpha, label='output $y_\\alpha$')
ax[0].set_ylabel('Amplitude')
ax[0].set_title('α-axis signal')
ax[0].legend(); ax[0].grid(True)

ax[1].plot(t, u_beta, label='input $u_\\beta$')
ax[1].plot(t, y_beta, label='output $y_\\beta$')
ax[1].set_xlabel('Time [s]')
ax[1].set_ylabel('Amplitude')
ax[1].set_title('β-axis signal')
ax[1].legend(); ax[1].grid(True)

fig.suptitle(f'VRAF extraction of {f_harm} Hz ({harm_seq}-sequence)', fontsize=14)
fig.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()



''''''
vraf = VRAF(
    f_shift = 1000,   # 逆相 1 kHz を抽出
    Fs      = 12000,  # 12 kHz 以上を推奨
    seq     = "neg",  # ＜ここだけ切り替える＞
    fc      = 150,    # デモジュ後の DC 近傍だけ通す
    order   = 4       # 偶数次数
)
''''''