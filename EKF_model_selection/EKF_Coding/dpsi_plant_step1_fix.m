
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
Rs = 37e-3;          % stator resistance [Ω]
Ts = 500e-6;         % sample time [s] (200 µs)

% Q = diag([1e-4, 1e-4, 1e-9, 1e-9]); % process noise
Q = diag([1e-4, 1e-4, 1e-12, 1e-12]); % process noise
R = diag([1e-3, 1e-3]);             % sensor noise

H = [1 0 0 0;                       % measurement matrix
     0 1 0 0];

% ── EKF – 1) prediction ─────────────────────────────────────────
u = [vd; vq; omega];

% LUTが0を出力した場合のゼロ除算を回避
Ld_map = max(Ld_map, 1e-6);
Lq_map = max(Lq_map, 1e-6);

x_pred = f_discrete_delta_phi(x_est, u, Rs, Ts, ...
                              phi_d_map, phi_q_map, Ld_map, Lq_map);

F = calculate_F_delta_phi(x_est, u, Rs, Ts, ...
                          Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

P_pred = F*P_est*F.' + Q;

% ── EKF – 2) update ─────────────────────────────────────────────
z   = [id_meas; iq_meas];
y   = z - H*x_pred;                % innovation
% S   = H*P_pred*H.' + R;
% K   = P_pred*H.'/S;                % Kalman gain
S = H*P_pred*H.' + R;
K = (P_pred*H.') / S;   % バックスラッシュ(\)よりも読みやすい

x_est = x_pred + K*y;
P_est = (eye(4) - K*H)*P_pred;

% --- 共分散更新 (Joseph) ---
I_KH  = eye(5) - K*H;
P_est = I_KH * P_pred * I_KH.' + K * R * K.';

% ── outputs ─────────────────────────────────────────────────────
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
Id_est          = x_est(1);
Iq_est          = x_est(2);
P_diag          = diag(P_est);
end



% 状態予測関数
function x_next = f_discrete_delta_phi(x, u, Rs, Ts, phi_d_map, phi_q_map, Ld_map, Lq_map)
    id      = x(1);
    iq      = x(2);
    delta_phi_d = x(3);
    delta_phi_q = x(4);

    vd      = u(1);
    vq      = u(2);
    omega   = u(3);

    % 電流の微分方程式
    did_dt = (1/Ld_map) * (vd - Rs*id + omega*(phi_q_map + delta_phi_q));
    diq_dt = (1/Lq_map) * (vq - Rs*iq - omega*(phi_d_map + delta_phi_d));
    
    % 状態の更新 (Δφはランダムウォークなので変化しない)
    id_next = id + Ts * did_dt;
    iq_next = iq + Ts * diq_dt;
    delta_phi_d_next = delta_phi_d;
    delta_phi_q_next = delta_phi_q;

    x_next = [id_next; iq_next; delta_phi_d_next; delta_phi_q_next];
end

% ヤコビ行列Fの計算関数
function F = calculate_F_delta_phi(x, u, Rs, Ts, Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq)
    omega = u(3);

    % 連続時間系のヤコビ行列A
    A = zeros(4,4);
    A(1,1) = (-Rs + omega * Lqd) / Ld_map;
    A(1,2) = (omega * Lqq) / Ld_map;
    A(1,4) = omega / Ld_map;
    
    A(2,1) = (-omega * Ldd) / Lq_map;
    A(2,2) = (-Rs - omega * Ldq) / Lq_map;
    % A(2,2) = (-Rs - omega * Ldq) / Lq_map; 

    A(2,3) = -omega / Lq_map;

    % 離散化: F = I + A*Ts (1次近似)
    F = eye(4) + A * Ts;
end




% P_est = (eye(4) - K*H)*P_pred; % ← 数値的に不安定な標準形式

% Joseph形式による共分散更新 - 数値的に安定
I_KH = eye(4) - K*H;
P_est = I_KH * P_pred * I_KH.' + K * R * K.';