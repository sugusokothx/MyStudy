function [delta_phi_d_est, delta_phi_q_est, P_diag, Id_est, Iq_est] = ...
    ekf_delta_phi_estimator_Rs( ...
        id_meas, iq_meas, vd, vq, omega, ...
        phi_d_map, phi_q_map, ...
        Ld_map, Lq_map, ...
        Ldd, Ldq, Lqd, Lqq) %#codegen
%────────────────────────────────────────────────────────
%   5-state EKF (id, iq, Δφd, Δφq, Rs) – C-friendly version
%
%   ※ 依存ユーティリティ (同一フォルダに置くこと)
%      ├─ mat22_*  (mul, add, sub, inv)
%      ├─ mat25_mul,  mat52_mul
%      ├─ mat55_*  (mul, add, eye)
%      └─ matT_transpose (22, 25, 52, 55)
%────────────────────────────────────────────────────────
% constants
Rs_nom = 37e-3;
Rs_min = 0.7 * Rs_nom;
Rs_max = 1.3 * Rs_nom;
Ts     = coder.const(500e-6);

% persistent state
persistent x_est P_est
if isempty(x_est)
    x_est = zeros(5,1);
    x_est(5) = Rs_nom;
    P_est = mat55_diag([0.01, 0.01, 1e-2, 1e-2, (0.3*Rs_nom)^2]);
end

% covariance (const, pre-built 5×5 / 2×2)
Q = mat55_diag([1e-4, 1e-4, 1e-12, 1e-12, 1e-10]);
R = mat22_diag([1e-3, 1e-3]);

% measurement matrix H (2×5)
H = [1 0 0 0 0;
     0 1 0 0 0];

%────────────────── prediction ──────────────────
u = [vd; vq; omega];

Ld_map = max(Ld_map, 1e-6);
Lq_map = max(Lq_map, 1e-6);

x_pred = f_discrete_delta_phiRs(x_est, u, Ts, ...
                                phi_d_map, phi_q_map, Ld_map, Lq_map, ...
                                Rs_min, Rs_max);

F = calculate_F_delta_phiRs(x_est, u, Ts, ...
                            Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

% P_pred = F*P_est*F.' + Q ← ユーティリティで展開
P_tmp  = mat55_mul(F, P_est);                 % 5×5
P_pred = mat55_add(mat55_mul(P_tmp, mat55_transpose(F)), Q);

%────────────────── update ──────────────────────
z = [id_meas; iq_meas];                       % 2×1
Hx = mat25_vec5_mul(H, x_pred);               % 2×1
y  = vec2_sub(z, Hx);                         % innovation (2×1)

HP     = mat25_mul(H, P_pred);                % 2×5
S      = mat22_add(mat25_mul(HP, mat52_transpose(H)), R); % 2×2
S_inv  = mat22_inv(S);                        % 2×2

PHT    = mat55_mul(P_pred, mat52_transpose(H)); % 5×2
K      = mat52_mul(PHT, S_inv);               % 5×2   Kalman gain

dx     = mat52_vec2_mul(K, y);                % 5×1
x_est  = vec5_add(x_pred, dx);                % a priori + correction

I_KH   = mat55_eye();                         % 5×5
KH     = mat55_mul(K, H);                     % 5×5
I_KH   = mat55_sub(I_KH, KH);                 % I - K*H
P_est  = mat55_mul(I_KH, P_pred);             % Joseph 近似で可

%────────────────── outputs ──────────────────────
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
Id_est          = x_est(1);
Iq_est          = x_est(2);
P_diag          = [P_est(1,1); P_est(2,2); P_est(3,3); ...
                   P_est(4,4); P_est(5,5)];
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


% ===================== mat55_mul.m =========================
function C = mat55_mul(A,B) %#codegen
C = zeros(5,5);
for i=1:5
    for j=1:5
        s = 0.0;
        for k=1:5
            s = s + A(i,k)*B(k,j);
        end
        C(i,j) = s;
    end
end
end

% ===================== mat25_mul.m (2×5 * 5×5) = 2×5 =========
function C = mat25_mul(A,B) %#codegen
C = zeros(2,5);
for i=1:2
    for j=1:5
        s = 0.0;
        for k=1:5
            s = s + A(i,k)*B(k,j);
        end
        C(i,j) = s;
    end
end
end

% ===================== mat52_vec2_mul.m =====================
function y = mat52_vec2_mul(A,x) %#codegen
% A:5×2, x:2×1 → y:5×1
y = zeros(5,1);
for i=1:5
    y(i) = A(i,1)*x(1) + A(i,2)*x(2);
end
end

% ===================== mat22_inv.m ==========================
function B = mat22_inv(A) %#codegen
det = A(1,1)*A(2,2) - A(1,2)*A(2,1);
inv_det = 1.0 / det;
B = [ A(2,2), -A(1,2); ...
     -A(2,1),  A(1,1)] * inv_det;
end
