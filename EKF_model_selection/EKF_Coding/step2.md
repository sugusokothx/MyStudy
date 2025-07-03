### STEP 2 Δφ-EKF — **map-inputs version**

すべてのマップ値（基準磁束・1次導関数・2次導関数）を **引数** として受け取る形に改修しました。
Simulink では Lookup Table → Bus → EKF ブロックに配線するだけで、ルックアップ関数呼び出しは一切不要になります。

---

```matlab
%====================================================================
% ekf_ipm_delta_phi.m  – STEP-2 (Δφ, 2nd-order) EKF
%    • Analytic Jacobian / Hessian version
%    • All map values are supplied from Simulink inputs
% -------------------------------------------------------------------
%  Inputs (scalars unless noted)
%   id_meas, iq_meas           [A]
%   vd, vq                     [V]
%   omega                      [rad/s]
%
%   phi_d0, phi_q0             [Wb]      – base flux linkage
%   Ldd, Ldq, Lqd, Lqq         [H]       – first-derivative map (∂φ/∂i)
%   Hd(2×2), Hq(2×2)           [H/A]     – Hessian matrices
%
%  Outputs
%   dphi_d_est, dphi_q_est     [Wb]      – Δφ estimates
%   id_est,   iq_est           [A]       – filtered currents
%   P_diag                               – diag(P) for logging
%====================================================================
function [dphi_d_est, dphi_q_est, id_est, iq_est, P_diag] = ...
    ekf_ipm_delta_phi(id_meas, iq_meas, vd, vq, omega, ...
                      phi_d0, phi_q0, ...
                      Ldd, Ldq, Lqd, Lqq, ...
                      Hd,  Hq)

%% ── persistent variables ─────────────────────────────────────────
persistent x_est P_est
if isempty(x_est)
    x_est = [0; 0; 0; 0];                % [id iq Δφd Δφq]
    P_est = diag([10, 10, 1e-6, 1e-6]);
end

%% ── EKF tunable constants ───────────────────────────────────────
Rs = 30e-3;                % stator resistance [Ω]
Ts = 1/5000;               % sample time [s]

Q_i   = (0.5)^2;           % A²
Q_phi = 1e-10;             % Wb²
Q = diag([Q_i,Q_i,Q_phi,Q_phi]);

R = diag([(0.2)^2, (0.2)^2]);   % current-sensor noise

H = [1 0 0 0;
     0 1 0 0];

%% ── 1) prediction ───────────────────────────────────────────────
u = [vd; vq; omega];

[x_pred, J_inv, phi_dot] = ...
          f_discrete_delta_phi(x_est, u, Rs, Ts, ...
                               phi_d0, phi_q0, ...
                               Ldd, Ldq, Lqd, Lqq);

F = calculate_F_delta_phi_analytic(x_est, u, Rs, Ts, ...
                                   J_inv, phi_dot, ...
                                   Ldd, Ldq, Lqd, Lqq, ...
                                   Hd,  Hq);

P_pred = F*P_est*F.' + Q;

%% ── 2) update ───────────────────────────────────────────────────
z = [id_meas; iq_meas];
y = z - H*x_pred;                 % innovation
S = H*P_pred*H.' + R;
K = P_pred*H.'/S;                 % Kalman gain

x_est = x_pred + K*y;
P_est = (eye(4)-K*H)*P_pred;

%% ── outputs ─────────────────────────────────────────────────────
dphi_d_est = x_est(3);
dphi_q_est = x_est(4);
id_est     = x_est(1);
iq_est     = x_est(2);
P_diag     = diag(P_est);
end
```

---

#### Helper 1: 状態予測

```matlab
function [x_next, J_inv, phi_dot] = ...
         f_discrete_delta_phi(x, u, Rs, Ts, ...
                              phi_d0, phi_q0, ...
                              Ldd, Ldq, Lqd, Lqq)

id = x(1);  iq = x(2);
dphid = x(3);  dphiq = x(4);

vd = u(1);  vq = u(2);  omega = u(3);

% ── Jacobian (first-derivative) matrix ─────────────────────────
J = [Ldd, Ldq;
     Lqd, Lqq];
J_inv = inv(J);

% ── actual flux linkage ───────────────────────────────────────
phi_d = phi_d0 + dphid;
phi_q = phi_q0 + dphiq;

% ── flux derivative (voltage eq.) ─────────────────────────────
phi_dot_d = vd - Rs*id + omega*phi_q;
phi_dot_q = vq - Rs*iq - omega*phi_d;
phi_dot   = [phi_dot_d; phi_dot_q];

% ── current derivative ────────────────────────────────────────
idiq_dot = J_inv * phi_dot;

id_next = id + Ts*idiq_dot(1);
iq_next = iq + Ts*idiq_dot(2);

x_next = [id_next; iq_next; dphid; dphiq];
end
```

---

#### Helper 2: 解析ヤコビ行列

```matlab
function F = calculate_F_delta_phi_analytic(x, u, Rs, Ts, ...
                                            J_inv, phi_dot, ...
                                            Ldd, Ldq, Lqd, Lqq, ...
                                            Hd,  Hq)
% Analytic F = ∂f/∂x   (4×4)

id = x(1);  iq = x(2);   %#ok<NASGU>
vd = u(1);  vq = u(2);  omega = u(3); %#ok<NASGU>

% ── pre-compute repeatedly used terms ─────────────────────────
w = J_inv * phi_dot;          % [did/dt ; diq/dt]

% ── dJ/d(state) tensors (only wrt id & iq) ────────────────────
dJ_did = [Hd(1,1), Hd(1,2);
          Hq(1,1), Hq(1,2)];

dJ_diq = [Hd(2,1), Hd(2,2);
          Hq(2,1), Hq(2,2)];

% ── d(phi_dot)/d(state) matrix (2×4) ──────────────────────────
DP = zeros(2,4);
DP(:,1) = [-Rs + omega*Lqd;       -omega*Ldd ]; % ∂/∂id
DP(:,2) = [ omega*Lqq;            -Rs - omega*Ldq ]; % ∂/∂iq
DP(:,3) = [ 0;                    -omega ];          % ∂/∂Δφd
DP(:,4) = [ omega;                0      ];          % ∂/∂Δφq

% ── build F ───────────────────────────────────────────────────
F = eye(4);

for idx = 1:4
    switch idx
        case 1, dJdx = dJ_did;
        case 2, dJdx = dJ_diq;
        otherwise, dJdx = zeros(2,2);  % Δφ → J independent
    end

    dwdx = -J_inv*(dJdx*w) + J_inv*DP(:,idx);

    F(1,idx) = 1 + Ts*dwdx(1);
    F(2,idx) =     Ts*dwdx(2);
end
end
```

---

## Simulink 配線例

| Port                                                       | 内容    | 典型的 2-D Lookup Table |
| ---------------------------------------------------------- | ----- | -------------------- |
| `phi_d0`, `phi_q0`                                         | φ-map | **φ(id,iq)**         |
| `Ldd`, `Ldq`, `Lqd`, `Lqq`                                 | 一次導関数 | **∂φ/∂i** LUT        |
| `Hd(1,1), Hd(1,2); Hd(2,2)`<br>`Hq(1,1), Hq(1,2); Hq(2,2)` | 二次導関数 | **Hessian** LUT      |

1. 各 LUT 出力を **Bus Creator** でまとめて EKF ブロックへ。
2. `Hd` と `Hq` は *2×2 Matrix Signal* として扱うか、上三角成分を 3-要素ベクトルで渡し、ブロック内部で再構築してもかまいません（上記コードは 2×2 行列を想定）。
3. STEP-1 と STEP-2 を置き換えるだけで性能比較が可能です。

これで STEP-2 も完全に **外部マップ入力対応** になりました。
さらなる検証（収束スピード、数値安定性、Q/R チューニングなど）が必要になったら、いつでもご相談ください！
