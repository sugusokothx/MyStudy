import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, group_delay

# -----------------------------
# 1)  VRAF with phase compensation
# -----------------------------
class VRAFPhaseComp:
    """
    Vector‑Rotator‑Assisted Filter with phase‑delay compensation.
    """
    def __init__(self, f_shift, Fs, seq='pos', fc=80, order=4, gd_manual=None):
        self.ω = 2 * np.pi * f_shift
        self.Fs = Fs
        self.seq = seq
        self.t_idx = 0

        # Butterworth LPF
        self.b, self.a = butter(order, fc / (Fs * 0.5), btype='low')

        # group delay (samples)
        if gd_manual is None:
            _, gd = group_delay((self.b, self.a), w=[0])
            self.gd_samp = float(gd[0])
        else:
            self.gd_samp = float(gd_manual)
        self.gd_sec = self.gd_samp / Fs

        n_state = max(len(self.a), len(self.b)) - 1
        self.zR = np.zeros(n_state)
        self.zI = np.zeros_like(self.zR)

    def reset(self):
        self.t_idx = 0
        self.zR[:] = 0
        self.zI[:] = 0

    def process(self, u_alpha, u_beta):
        u_p = u_alpha + 1j * u_beta
        t = self.t_idx / self.Fs
        self.t_idx += 1

        if self.seq == 'pos':
            u_tilde = u_p * np.exp(-1j * self.ω * t)
            rot_back = np.exp(+1j * self.ω * (t - self.gd_sec))
        else:
            u_tilde = u_p * np.exp(+1j * self.ω * t)
            rot_back = np.exp(-1j * self.ω * (t - self.gd_sec))

        xR, self.zR = lfilter(self.b, self.a, [u_tilde.real], zi=self.zR)
        xI, self.zI = lfilter(self.b, self.a, [u_tilde.imag], zi=self.zI)
        x_tilde = xR[0] + 1j * xI[0]

        y_p = x_tilde * rot_back
        return y_p.real, y_p.imag

# -----------------------------
# 2)  Simulation parameters
# -----------------------------
Fs = 12_000
Ts = 1 / Fs
Tend = 0.05  # 50 ms
t = np.arange(0, Tend, Ts)

f_target = 200   # [Hz]  positive‑sequence
f_noise  = 1000  # [Hz]  negative‑sequence

A_target = 4.0
A_noise  = 1.0

# build αβ input: target is +seq, noise is −seq
u_alpha = A_target * np.cos(2 * np.pi * f_target * t) + A_noise * np.cos(2 * np.pi * f_noise * t)
u_beta  = A_target * np.sin(2 * np.pi * f_target * t) - A_noise * np.sin(2 * np.pi * f_noise * t)  # negative‑seq for noise

# -----------------------------
# 3)  Run VRAF with phase comp
# -----------------------------
vraf_pc = VRAFPhaseComp(
    f_shift=f_target,
    Fs=Fs,
    seq='pos',
    fc=f_target＊0.4,
    order=4
)

y_alpha = np.zeros_like(u_alpha)
y_beta  = np.zeros_like(u_beta)

for k in range(len(t)):
    y_alpha[k], y_beta[k] = vraf_pc.process(u_alpha[k], u_beta[k])

# -----------------------------
# 4)  Plot results (two separate figures)
# -----------------------------
plt.figure()
plt.plot(t, u_alpha, label="input $u_\\alpha$")
plt.plot(t, y_alpha, label="output $y_\\alpha$")
plt.title("α‑axis signal (input vs. filtered)")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(t, u_beta, label="input $u_\\beta$")
plt.plot(t, y_beta, label="output $y_\\beta$")
plt.title("β‑axis signal (input vs. filtered)")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.grid(True)
plt.legend()

plt.show()
