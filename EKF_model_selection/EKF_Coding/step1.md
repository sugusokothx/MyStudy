### Δφ-EKF (1st-order) — refactored for *external* map inputs

Below is a drop-in replacement for your **STEP 1** MATLAB function.
All quantities that were previously fetched internally from `lookup_*` routines are now **explicit input arguments**, so the block can be wired directly to 2-D (or 3-D) Lookup Table blocks in Simulink.

```matlab
%==================================================================
% ekf_delta_phi_estimator.m  –  STEP-1 (Δφ, 1st-order) EKF
%    Map values are supplied from the outside (Simulink inputs).
%------------------------------------------------------------------
%  Inputs  (scalars, per control period)
%    id_meas, iq_meas         [A]   – current sensors
%    vd, vq                   [V]   – voltage commands
%    omega                    [rad/s] – electrical speed
%    phi_d_map,  phi_q_map    [Wb]  – base-flux LUT values
%    Ld_map,     Lq_map       [H]   – ∂φ/∂i   (first derivatives)
%    Ldd, Ldq, Lqd, Lqq       [H/A] – ∂²φ/∂i² (mixed derivatives)
%
%  Outputs
%    delta_phi_d_est, delta_phi_q_est   [Wb]  – Δφ estimates
%    P_diag                             …    – diag(P) for logging
%    Id_est, Iq_est                     [A]  – filtered currents
%==================================================================
function [delta_phi_d_est, delta_phi_q_est, P_diag, Id_est, Iq_est] = ...
    ekf_delta_phi_estimator(id_meas, iq_meas, vd, vq, omega, ...
                            phi_d_map, phi_q_map, ...
                            Ld_map, Lq_map, ...
                            Ldd, Ldq, Lqd, Lqq)

% ── persistent state ─────────────────────────────────────────────
persistent x_est P_est
if isempty(x_est)
    x_est = [0; 0; 0; 0];                   % [id iq Δφd Δφq]
    P_est = diag([0.01, 0.01, 1e-2, 1e-2]); % large uncertainty on Δφ
end

% ── constants (adjust if necessary) ──────────────────────────────
Rs = 30e-3;          % stator resistance [Ω]
Ts = 1/5000;         % sample time [s] (200 µs)

Q = diag([1e-4, 1e-4, 1e-9, 1e-9]); % process noise
R = diag([1e-3, 1e-3]);             % sensor noise

H = [1 0 0 0;                       % measurement matrix
     0 1 0 0];

% ── EKF – 1) prediction ─────────────────────────────────────────
u = [vd; vq; omega];

x_pred = f_discrete_delta_phi(x_est, u, Rs, Ts, ...
                              phi_d_map, phi_q_map, Ld_map, Lq_map);

F = calculate_F_delta_phi(x_est, u, Rs, Ts, ...
                          Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

P_pred = F*P_est*F.' + Q;

% ── EKF – 2) update ─────────────────────────────────────────────
z   = [id_meas; iq_meas];
y   = z - H*x_pred;                % innovation
S   = H*P_pred*H.' + R;
K   = P_pred*H.'/S;                % Kalman gain

x_est = x_pred + K*y;
P_est = (eye(4) - K*H)*P_pred;

% ── outputs ─────────────────────────────────────────────────────
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
Id_est          = x_est(1);
Iq_est          = x_est(2);
P_diag          = diag(P_est);
end
```

The *two* helper functions are **unchanged** (they already accept the map values as arguments):

```matlab
function x_next = f_discrete_delta_phi(x, u, Rs, Ts, ...
                                       phi_d_map, phi_q_map, ...
                                       Ld_map, Lq_map)
    % … (same code as before) …
end
```

```matlab
function F = calculate_F_delta_phi(x, u, Rs, Ts, ...
                                   Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq)
    % … (same code as before) …
end
```

---

## How to wire this in Simulink

| **Signal**                 | Typical source block                | Dimension |
| -------------------------- | ----------------------------------- | --------- |
| `phi_d_map`, `phi_q_map`   | 2-D Lookup Table (id, iq)           | 1×1       |
| `Ld_map`, `Lq_map`         | same LUT or pre-computed table      | 1×1       |
| `Ldd`, `Ldq`, `Lqd`, `Lqq` | 2-D Lookup Table (mixed derivative) | 1×1 each  |

1. **Group the eight map outputs** into a **Bus** (or simply keep them as separate ports) and feed them to the EKF block.
2. Ensure all Lookup Tables use **trigger-sampled** interpolation (sample-and-hold) so that map values stay constant over the control period.
3. Use **Data Store Memory** or a **Unit Delay** inside the EKF Subsystem if you prefer the persistent variables to live entirely within Simulink.

With this decoupling finished, you can now:

* Run STEP 1 in Simulink and check that `Δφ` converges.
* Duplicate the model, swap the EKF block for the STEP 2 analytic-Jacobian version, and compare convergence speed and covariance evolution.

Let me know if you need help building the LUTs or automating the comparative performance test!
