function [phi_d, phi_q] = estimate_flux_steady(id, iq, vd, vq, omega, Rs)
% estimate_flux_steady  Estimate dq‑axis flux linkages from a single
% sample of {id, iq, vd, vq, omega} assuming quasi‑steady state (φ̇ ≈ 0).
% The method rearranges the steady‑state stator voltage equations:
%   0 = vd − Rs*id + ω * φ_q  →  φ_q = (vd − Rs*id)/ω
%   0 = vq − Rs*iq − ω * φ_d  →  φ_d = −(vq − Rs*iq)/ω
% 
% Inputs
%   id, iq : measured dq currents  [A]
%   vd, vq : dq voltages applied   [V]
%   omega  : electrical angular speed [rad/s]
%   Rs     : stator resistance       [Ω]
% 
% Output
%   phi_d, phi_q : estimated flux linkages [Wb]
% 
% Notes
% * Requires |omega| > 0.1 rad/s to avoid division by zero. If omega is
%   below the threshold, the function returns NaN.
% * For transient conditions (φ̇ ≠ 0) this estimate is biased; use
%   integrate‑based methods instead.
% ---------------------------------------------------------------------

omega_thr = 0.1;  % rad/s threshold to avoid division by zero
if abs(omega) < omega_thr
    phi_d = NaN; phi_q = NaN;
    warning('estimate_flux_steady:omegaTooSmall', ...
        'Electrical speed too small (|ω| < %.2f rad/s). Returning NaNs.', omega_thr);
    return;
end

phi_q = (vd - Rs*id) / omega;
phi_d = -(vq - Rs*iq) / omega;
end
