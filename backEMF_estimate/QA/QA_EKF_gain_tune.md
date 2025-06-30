Below is a **check-list & tuning playbook** you can work through to make every state ( *i*<sub>d</sub>, *i*<sub>q</sub>, Δφ<sub>d/q</sub>, Δψ<sub>f</sub>, R<sub>s</sub> ) converge quickly and stay locked on the true values.

---

## 1 Start with the model—match reality before touching gains

| Item                                    | What to verify                                                                                                           | Typical fix if wrong                                                                                   |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------ |
| **Voltage model**                       | Are you using exactly the same sign convention and per-unit system as the plant?                                         | Keep one single “source of truth” function and import it in both plant and EKF files.                  |
| **Λ<sub>d/q</sub> LUT vs. inverse LUT** | Interpolation artefacts near the borders create large model error, which EKF interprets as process noise.                | Clip the input currents to the map’s valid rectangle or extend the map with a slim extrapolation band. |
| **θ-dependent cross coupling**          | In the plant, $dψ/dt$ uses the instantaneous ω·J·ψ term; make sure EKF’s Jacobian step sees the same sign and magnitude. | If necessary, call `numerical_jacobian` with a *relative* step: `eps = max(1e-6*abs(x[k]), 1e-8)`.     |

> **Take-away:** every systematic model mismatch ends up in the innovation; you will compensate it later with *Q*, but removing it up-front shortens the tuning loop drastically.

---

## 2 Tune the **process-noise matrix Q**

Think of *Q* as “how much can the real world surprise me between two samples?”
Bigger *Q* → EKF trusts the measurement more and adapts faster.

| State                            | What drives its “true” variation?                | Practical rule of thumb                                                                                                  |
| -------------------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ |
| *i*<sub>d</sub>, *i*<sub>q</sub> | Un-modelled PWM ripple, discretisation error     | Start at 10× `R_current` and dial down until the estimate stops chattering.                                              |
| Δφ<sub>d</sub>, Δφ<sub>q</sub>   | Slotting, saturation that the map didn’t capture | Set to ≈ (1–3 % of Φ<sub>n</sub>)² / (10 ms).                                                                            |
| Δψ<sub>f</sub>                   | Thermal demagnetisation, long-term ageing        | Start *very* small (1 × 10<sup>-10</sup>) and increase until you get the desired tracking bandwidth (minutes? seconds?). |
| R<sub>s</sub>                    | Copper temperature                               | Convert °C → Ω (e.g. 0.0039 Ω / °C) and choose *Q* so the filter can follow a worst-case dT/dt.                          |

> **Procedure:**
>
> 1. Freeze *R* at the real sensor noise level.
> 2. Increase *Q* for **one** state at a time until it just starts to follow the truth in your plots.
> 3. If a state oscillates, you went a bit too far → halve that entry.

---

## 3 Tune the **measurement-noise matrix R**

Measure your current sensor noise with the motor locked (or look up the spec).
Set *R* equal to the *variance*, not the standard deviation.

*Typical trap:* if you down-sample the ADC to 10 kHz but EKF rate is 1 kHz, the effective variance is 10× bigger.

---

## 4 Initial covariance *P₀* and state guess

*P₀* should represent “how wrong can I be at *t = 0*?”

| State          | Safe default                  |
| -------------- | ----------------------------- |
| currents       | $(I_{\max})^{2}$              |
| flux errors    | $(0.05 … 0.1 \text{ Wb})^{2}$ |
| Δψ<sub>f</sub> | same as above                 |
| R<sub>s</sub>  | $(0.3 R_{s,\text{nom}})^{2}$  |

Big *P₀* gives a big first-step gain and helps catch the state quickly, then shrinks automatically.

---

## 5 Observation model coupling: make sure Δφ & Δψ<sub>f</sub> actually affect *H P Hᵀ*

In your code, they do (via λ<sub>d/q</sub>) **only if** the motor is spinning (*ω ≠ 0*).
When you test at stand-still, the observability rank drops and those states freeze.

> **Tip:** inject a tiny virtual test signal (±1 rad s⁻¹) or run the first few hundred milliseconds with a ramp-up so the EKF can “see” the magnetic states.

---

## 6 Step-by-step tuning recipe

1. **Static sanity check**
   *Q* = diag(0)… → does the filter explode? Good! Abort after 1–2 ms.
2. **Currents only**
   Enable *Q* for *i*<sub>d/q</sub> until currents converge in ≤ 1 ms.
3. **Add R<sub>s</sub>**
   Turn on its *Q*, monitor convergence to the true 0.05 Ω.
4. **Add Δφ<sub>d/q</sub>**
   Increase their *Q* slowly; watch that they don’t cannibalise R<sub>s</sub>.
5. **Finally Δψ<sub>f</sub>**
   Raise *Q* until the blue estimate in the plot keeps up with the red “true” 10 % ramp without noise-driven drift when the ramp ends.
6. **Stress test**
   Change speed, load, sign of *i*<sub>d</sub>.
   If the filter lags → enlarge the corresponding *Q* element;
   if it jitters → reduce said *Q* or increase that row/col in *R*.

---

## 7 If tuning alone is not enough

* **Augment the state further** (L<sub>d</sub>, L<sub>q</sub>, mis-alignment angle, etc.).
* Switch to an **Unscented or Cubature KF** if linearisation is too coarse.
* Add a **white-noise acceleration** model: $\dot{Δψ_f} = w$ instead of zero for smoother tracking.

---

### TL;DR

1. **Match the physics** (same equations both sides).
2. **Tune Q from large to small** until each state just follows the truth.
3. **Set R to the real sensor variance—never “what feels good.”**
4. **Big *P₀* and a bit of rotor motion** guarantee early observability.

Work through this list systematically and every state trace will collapse onto the true curve. Good luck, and ping me when you want to re-factor or add the next layer of complexity!
