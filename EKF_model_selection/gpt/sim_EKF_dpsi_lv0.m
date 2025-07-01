function [dphi_d_est, dphi_q_est, id_est, iq_est, P_diag] = ekf_ipm_delta_phi(id_meas, iq_meas, vd, vq, omega)
% EKF implementation for Δφ approach (Approach B)
%  State vector x = [id; iq; dphi_d; dphi_q]
%  Measurements  z = [id_meas; iq_meas]
%  Control input  u = [vd; vq; omega]
%  Requires: flux_map_with_grad(id, iq) that returns
%            [phi_d0, phi_q0, Ldd, Ldq, Lqd, Lqq]
%  Author: ChatGPT (o3) 2025‑07‑02

%% parameter definitions (edit as necessary)
Rs = 30e-3;        % stator resistance [Ohm]
Ts = 1/5000;       % sampling period  [s]  (=200 µs)

% Process / measurement noise (tune for your system)
Q = diag([ (0.5)^2, (0.5)^2, 1e-10, 1e-10 ]);  % [A^2, A^2, Wb^2, Wb^2]
R = diag([ (0.2)^2, (0.2)^2 ]);                % sensor σ ≈0.2 A

% Measurement matrix H (constant)
H = [ 1 0 0 0 ;
      0 1 0 0 ];

%% persistent states
persistent x_est P_est
if isempty(x_est)
    x_est = zeros(4,1);       % [id iq Δφd Δφq]
    P_est = eye(4)*0.01;
end

%% pack inputs
z = [id_meas; iq_meas];
u = [vd; vq; omega];

%% --------------------  PREDICTION STEP  --------------------
[x_pred, Jphi] = f_discrete_delta_phi(x_est, u, Rs, Ts);
F = calculate_F_delta_phi(x_est, u, Rs, Ts, Jphi);
P_pred = F*P_est*F.' + Q;

%% --------------------  UPDATE STEP  -----------------------
y_pred = H * x_pred;                    % measurement prediction
S   = H * P_pred * H.' + R;             % innovation covariance
K   = P_pred * H.' / S;                 % Kalman gain
x_est = x_pred + K * (z - y_pred);      % state update
P_est = (eye(4) - K*H) * P_pred;        % covariance update

%% outputs & basic bounding (optional)
dphi_d_est = x_est(3);
dphi_q_est = x_est(4);
id_est     = x_est(1);
iq_est     = x_est(2);
P_diag     = diag(P_est);

end  % === end main ===

%% -----------------------------------------------------------
function [x_next, Jphi] = f_discrete_delta_phi(x, u, Rs, Ts)
% One‑step forward Euler prediction and returns φ‑Jacobian Jphi
% x = [id iq dφd dφq]
% u = [vd vq ω_e]

id  = x(1);  iq  = x(2);
dφd = x(3);  dφq = x(4);
vd  = u(1);  vq  = u(2);  omega = u(3);

% --- look‑up base flux map and Jacobian (user‑provided function) ---
[φd0, φq0, Ldd, Ldq, Lqd, Lqq] = flux_map_with_grad(id, iq);
Jphi = [Ldd, Ldq; Lqd, Lqq];      % 2×2

% real fluxes
φd = φd0 + dφd;
φq = φq0 + dφq;

% φ dot (voltage equations)
φd_dot = vd - Rs*id + omega*φq;
φq_dot = vq - Rs*iq - omega*φd;

% i dot via inverse Jphi
id_iq_dot = Jphi \ [φd_dot; φq_dot];   % 2×1 solve

% Euler integration
id_next  = id  + Ts * id_iq_dot(1);
iq_next  = iq  + Ts * id_iq_dot(2);
% Δφ are random walk (process noise handled in Q)
dφd_next = dφd;
dφq_next = dφq;

x_next = [id_next; iq_next; dφd_next; dφq_next];
end

%% -----------------------------------------------------------
function F = calculate_F_delta_phi(x, u, Rs, Ts, Jphi)
% Numerical Jacobian of f_discrete_delta_phi w.r.t state x
% For 4‑state system: finite‑difference (central)

n = numel(x);
F = eye(n);

eps = 1e-6;  % perturbation size
for i = 1:n
    dx = zeros(n,1);
    dx(i) = eps;
    xp = f_discrete_delta_phi(x+dx, u, Rs, Ts);
    xm = f_discrete_delta_phi(x-dx, u, Rs, Ts);
    F(:,i) = (xp - xm) / (2*eps);
end
end

%% -----------------------------------------------------------
function [φd0, φq0, Ldd, Ldq, Lqd, Lqq] = flux_map_with_grad(id, iq)
% --- USER MUST IMPLEMENT / REPLACE WITH LUT ACCESS ---
% Placeholder linear model for compiling & unit‑test purpose only.
Ld_nom = 0.3e-3;  Lq_nom = 0.5e-3;  ψf = 0.08;  % example constants
φd0 = ψf + Ld_nom * id;          % simplistic linear map
φq0 =           Lq_nom * iq;
% Jacobian entries
Ldd = Ld_nom;   Ldq = 0;
Lqd = 0;        Lqq = Lq_nom;
end
