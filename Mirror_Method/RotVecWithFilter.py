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
vraf = VRAF(f_shift=f_harm, Fs=Fs, seq=harm_seq, fc=150, order=4)

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
