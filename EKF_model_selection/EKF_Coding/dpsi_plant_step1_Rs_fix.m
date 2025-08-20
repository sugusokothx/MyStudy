%==================================================================
% ekf_delta_phi_estimator_Rs.m – STEP-1+ (Δφ + Rs) EKF
%==================================================================
function [delta_phi_d_est, delta_phi_q_est, P_diag, Id_est, Iq_est] = ...
    ekf_delta_phi_estimator_Rs(id_meas, iq_meas, vd, vq, omega, ...
                               phi_d_map, phi_q_map, ...
                               Ld_map, Lq_map, ...
                               Ldd, Ldq, Lqd, Lqq)

% ── constants ────────────────────────────────────────────────────
Rs_nom = 37e-3;                      % nominal stator resistance [Ω]
Rs_min = 0.7 * Rs_nom;               % −30 %
Rs_max = 1.3 * Rs_nom;               % +30 %
Ts     = 500e-6;                     % sample time [s]

% ── persistent state ────────────────────────────────────────────
persistent x_est P_est
if isempty(x_est)
    % [id iq Δφd Δφq Rs]
    x_est = [0; 0; 0; 0; Rs_nom];
    P_est = diag([0.01, 0.01, 1e-2, 1e-2,  (0.3*Rs_nom)^2]); % Rs ±30 %
end

% ── covariance matrices ─────────────────────────────────────────
Q = diag([1e-4, 1e-4, 1e-12, 1e-12, 1e-10]);   % process noise
R = diag([1e-3, 1e-3]);                        % sensor noise

% ── measurement matrix ──────────────────────────────────────────
H = [1 0 0 0 0;
     0 1 0 0 0];

% ── EKF – 1) prediction ─────────────────────────────────────────
u = [vd; vq; omega];

Ld_map = max(Ld_map, 1e-6);   % avoid divide-by-zero
Lq_map = max(Lq_map, 1e-6);

x_pred = f_discrete_delta_phiRs(x_est, u, Ts, ...
                                phi_d_map, phi_q_map, Ld_map, Lq_map, ...
                                Rs_min, Rs_max);

F = calculate_F_delta_phiRs(x_est, u, Ts, ...
                            Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

P_pred = F*P_est*F.' + Q;

% ── EKF – 2) update ─────────────────────────────────────────────
z = [id_meas; iq_meas];
y = z - H*x_pred;                       % innovation
S = H*P_pred*H.' + R;
K = (P_pred*H.') / S;                   % Kalman gain

x_est = x_pred + K*y;
% P_est = (eye(5) - K*H) * P_pred;
% --- 共分散更新 (Joseph) ---
I_KH  = eye(5) - K*H;
P_est = I_KH * P_pred * I_KH.' + K * R * K.';


% ── outputs ─────────────────────────────────────────────────────
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
Id_est          = x_est(1);
Iq_est          = x_est(2);
P_diag          = diag(P_est);

% (Rs_est  = x_est(5);  % ←必要なら取得)
end

%==================================================================
% 状態予測関数
%==================================================================
function x_next = f_discrete_delta_phiRs(x, u, Ts, ...
                                         phi_d_map, phi_q_map, ...
                                         Ld_map, Lq_map, ...
                                         Rs_min, Rs_max)
    id      = x(1);   iq   = x(2);
    dphi_d  = x(3);   dphi_q = x(4);
    Rs      = x(5);

    vd = u(1);  vq = u(2);  omega = u(3);

    % 電流微分方程式
    did_dt = (1/Ld_map) * (vd - Rs*id + omega*(phi_q_map + dphi_q));
    diq_dt = (1/Lq_map) * (vq - Rs*iq - omega*(phi_d_map + dphi_d));

    % 離散時間更新（Δφ, Rs はランダムウォーク）
    id_next     = id     + Ts * did_dt;
    iq_next     = iq     + Ts * diq_dt;
    dphi_d_next = dphi_d;      % random walk
    dphi_q_next = dphi_q;
    Rs_next     = Rs;          % random walk

    % 物理上限でクリップ（±30 %）
    Rs_next = min(max(Rs_next, Rs_min), Rs_max);

    x_next = [id_next; iq_next; dphi_d_next; dphi_q_next; Rs_next];
end

%==================================================================
% ヤコビ行列 F の計算関数
%==================================================================
function F = calculate_F_delta_phiRs(x, u, Ts, ...
                                     Ld_map, Lq_map, ...
                                     Ldd, Ldq, Lqd, Lqq)
    id = x(1);  iq = x(2);  Rs = x(5);
    omega = u(3);

    % 連続時間系ヤコビ A (5×5)
    A = zeros(5,5);

    % ∂(did_dt)/∂(...)
    A(1,1) = (-Rs + omega*Lqd) / Ld_map;
    A(1,2) = ( omega*Lqq )     / Ld_map;
    A(1,4) =  omega            / Ld_map;
    A(1,5) = -id               / Ld_map;   % new: d/dRs

    % ∂(diq_dt)/∂(...)
    A(2,1) = (-omega*Ldd)      / Lq_map;
    A(2,2) = (-Rs - omega*Ldq) / Lq_map;
    A(2,3) = -omega            / Lq_map;
    A(2,5) = -iq               / Lq_map;   % new: d/dRs

    % Δφ̇, Rṡ  are random walks → zeros elsewhere

    % 離散化 (Euler 1st-order): F = I + A*Ts
    F = eye(5) + A * Ts;
end
