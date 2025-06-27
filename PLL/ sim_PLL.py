#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase-Compensated VRAF  +  HFI-PLL Demo
--------------------------------------

● HF injection : 1 kHz (γ-axis, negative-sequence)
● Demod filter : VRAF **with group-delay compensation** (PhaseComp)
● Plant model  : Rs-Ld/Lq, electrical speed ω_true
● Estimator    : PI-PLL  (Δ-axis current → phase error)

複素数演算・scipy.signal.butter/lfilter/group_delay すべて使用可。
そのまま実行すれば遅れゼロのトラッキングが確認できます。
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, group_delay


# ───────────────────────────────────────────────
# 0)  Phase-Compensated VRAF
# ───────────────────────────────────────────────
class VRAFPhaseComp:
    """
    Vector-Rotator-Assisted Filter with **phase-delay compensation**.
    回転位相を group-delay (gd_samp) サンプル分だけ前倒しして
    “入力＝出力時刻” を実現する。
    """

    def __init__(self, f_shift, Fs, seq='pos',
                 fc=150.0, order=4, gd_manual=None):
        self.Fs   = Fs
        self.seq  = seq
        self.ω    = 2*np.pi*f_shift
        self.t_idx = 0

        # Butterworth LPF
        self.b, self.a = butter(order, fc/(Fs*0.5), btype='low')
        n_state = max(len(self.a), len(self.b)) - 1
        self.zR = np.zeros(n_state)
        self.zI = np.zeros_like(self.zR)

        # --- group delay [samples] ---
        if gd_manual is None:
            # group_delay returns (w,g); take DC point
            _, gd = group_delay((self.b, self.a), w=[0])
            self.gd_samp = float(gd[0])
        else:
            self.gd_samp = float(gd_manual)

    # ---------------------------------------------------------
    def reset(self):
        self.t_idx = 0
        self.zR[:] = 0.0
        self.zI[:] = 0.0

    def process(self, u_alpha, u_beta):
        """
        Filter one αβ sample → extracted αβ at **same physical time**.
        Compensation: rotate-back uses (t_idx − gd)/Fs instead of t_idx/Fs.
        """
        u_p = u_alpha + 1j*u_beta
        t = self.t_idx / self.Fs
        self.t_idx += 1

        if self.seq == 'pos':
            u_tilde = u_p * np.exp(-1j*self.ω*t)
            sign = +1
        else:                           # negative-sequence
            u_tilde = u_p * np.exp(+1j*self.ω*t)
            sign = -1

        # IIR low-pass
        xR, self.zR = lfilter(self.b, self.a, [u_tilde.real], zi=self.zR)
        xI, self.zI = lfilter(self.b, self.a, [u_tilde.imag], zi=self.zI)
        x_tilde = xR[0] + 1j*xI[0]

        # ----------- phase-delay compensation -----------
        # rotate back with time (t − gd)
        t_back = (self.t_idx - self.gd_samp) / self.Fs
        rot_back = np.exp(sign*1j*self.ω*t_back)
        y_p = x_tilde * rot_back
        return y_p.real, y_p.imag


# ───────────────────────────────────────────────
# 1)  Simulation parameters
# ───────────────────────────────────────────────
Va   = 25.0          # HF injection amplitude [V]
f_h  = 1_000.0       # HF frequency [Hz]
ω_h  = 2*np.pi*f_h

Ld, Lq = 1.0e-3, 1.4e-3
R_s = 0.03

Ts   = 1/6_000*2     # ≈3.33e-4 s  (Fs ≈ 3 kHz)
Fs   = 1.0/Ts
Tend = 0.05

# PLL gains (will be retuned after delay-free demod)
Kp, Ki = 400.0, 4.0e5

# True motor speed (2000 rpm, 4 pole-pairs)
rpm  = 2_000
Pn   = 4
ω_true = rpm/60 * 2*np.pi * Pn

θ_est = np.deg2rad(30.0)
ω_est = 0.0

# Phase-comp VRAF  (negative-sequence extraction)
# vraf = VRAFPhaseComp(f_shift=f_h, Fs=Fs, seq='neg', fc=150.0, order=4)
vraf = VRAFPhaseComp(f_shift=f_h, Fs=Fs, seq='pos', fc=150.0, order=4)

# ───────────────────────────────────────────────
# 2)  Helpers
# ───────────────────────────────────────────────
def wrap(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def rot(alpha, vec):
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([ c*vec[0] - s*vec[1],
                      s*vec[0] + c*vec[1] ])


# ───────────────────────────────────────────────
# 3)  Logs
# ───────────────────────────────────────────────
t_log, θ_est_log, θ_true_log, err_deg_log = [], [], [], []


# ───────────────────────────────────────────────
# 4)  Main loop
# ───────────────────────────────────────────────
Id = Iq = 0.0
int_err = 0.0
θ_true = 0.0

steps = int(Tend / Ts)
for k in range(steps):
    t = k * Ts

    # (1) HF voltage injection (γ-axis sin)
    Vγ = Va * np.sin(ω_h*t)
    Vδ = 0.0
    Vα, Vβ = rot(θ_est, np.array([Vγ, Vδ]))

    # (2) dq current model
    Vd, Vq = rot(-θ_true, np.array([Vα, Vβ]))
    dId = (Vd - R_s*Id + ω_true*Lq*Iq) / Ld
    dIq = (Vq - R_s*Iq - ω_true*Ld*Id) / Lq
    Id += dId*Ts
    Iq += dIq*Ts
    Iα, Iβ = rot(θ_true, np.array([Id, Iq]))

    # (3) Phase-comp VRAF
    Ih_α, Ih_β = vraf.process(Iα, Iβ)

    # αβ → γδ using *delay-free* θ_est
    Ih_γ, Ih_δ = rot(-θ_est, np.array([Ih_α, Ih_β]))

    # (4) PLL
    err      = -Ih_δ              # negative-seq ⇒ −Δ current
    int_err += err * Ts
    ω_est    = Kp*err + Ki*int_err

    # (5) θ_est update
    θ_est += ω_est * Ts
    θ_est = wrap(θ_est)

    # (0) True angle update (sim only)
    θ_true += ω_true*Ts
    θ_true = wrap(θ_true)

    # Logs
    t_log.append(t)
    θ_est_log.append(np.rad2deg(θ_est))
    θ_true_log.append(np.rad2deg(θ_true))
    err_deg_log.append(np.rad2deg(wrap(θ_est - θ_true)))

# ───────────────────────────────────────────────
# 5)  Plot
# ───────────────────────────────────────────────
plt.figure(figsize=(9,4))
plt.plot(t_log, θ_est_log,  label=r'$\hat\theta_{\mathrm{est}}$')
plt.plot(t_log, θ_true_log, label=r'$\theta_{\mathrm{true}}$')
plt.xlabel('Time [s]'); plt.ylabel('Electrical angle [deg]')
plt.title('Phase tracking (Phase-Comp VRAF + Rs & cross)')
plt.grid(); plt.legend()

plt.figure(figsize=(9,4))
plt.plot(t_log, err_deg_log)
plt.xlabel('Time [s]'); plt.ylabel('Phase error [deg]')
plt.title('Phase error convergence – delay-free')
plt.grid(); plt.tight_layout(); plt.show()
