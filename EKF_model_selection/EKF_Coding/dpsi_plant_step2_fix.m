%====================================================================
% ekf_ipm_delta_phi.m  – STEP-2 (Δφ, 2nd-order) EKF
%    • ヘシアン行列を用いた解析的なヤコビ行列バージョン
%    • 全てのマップ値はSimulink等から入力として与えられる
% -------------------------------------------------------------------
%  Inputs (スカラーでなければ注記)
%   id_meas, iq_meas           [A]       - 電流の測定値
%   vd, vq                     [V]       - 電圧の指令値
%   omega                      [rad/s]   - 電気角速度
%
%   phi_d0, phi_q0             [Wb]      - 基準磁束マップ値
%   Ldd, Ldq, Lqd, Lqq         [H]       - 1階微分マップ (∂φ/∂i)
%   Hd(2×2), Hq(2×2)           [H/A]     - ヘシアン行列 (2階微分)
%
%  Outputs
%   dphi_d_est, dphi_q_est     [Wb]      - Δφの推定値
%   id_est,   iq_est           [A]       - フィルタリングされた電流
%   P_diag                     [various] - 共分散行列Pの対角成分 (ログ用)
%====================================================================
function [dphi_d_est, dphi_q_est, id_est, iq_est, P_diag] = ...
    ekf_ipm_delta_phi(id_meas, iq_meas, vd, vq, omega, ...
                      phi_d0, phi_q0, ...
                      Ldd, Ldq, Lqd, Lqq, ...
                      Hd,  Hq)

% ── 永続変数 (フィルタの状態を保持) ──────────────────────────────────
persistent x_est P_est
if isempty(x_est)
    % 状態ベクトルx = [id; iq; Δφd; Δφq]
    x_est = [0; 0; 0; 0];
    % 初期共分散行列P
    P_est = diag([10, 10, 1e-6, 1e-6]);
end

% ── EKFチューニング定数 ─────────────────────────────────────────────
Rs = 60e-3;                % 固定子抵抗 [Ω]
Ts = 500e-6;               % サンプリング時間 [s]

% プロセスノイズ共分散行列Q
Q_i   = (0.5)^2;           % 電流モデルの不確かさ [A²]
Q_phi = 1e-10;             % Δφのランダムウォークの大きさ [Wb²]
Q = diag([Q_i, Q_i, Q_phi, Q_phi]);

% 観測ノイズ共分散行列R
R = diag([(0.2)^2, (0.2)^2]);   % 電流センサのノイズ [A²]

% 観測行列H
H = [1 0 0 0;
     0 1 0 0];

% ── 1) 予測ステップ ────────────────────────────────────────────────
% 入力ベクトルuを定義
u = [vd; vq; omega];

% 状態予測: 1ステップ先の状態x_predを計算
[x_pred, J_inv, phi_dot] = ...
          f_discrete_delta_phi(x_est, u, Rs, Ts, ...
                               phi_d0, phi_q0, ...
                               Ldd, Ldq, Lqd, Lqq);

% ヤコビ行列Fの計算
F = calculate_F_delta_phi_analytic(x_est, u, Rs, Ts, ...
                                   J_inv, phi_dot, ...
                                   Ldd, Ldq, Lqd, Lqq, ...
                                   Hd,  Hq);

% 予測誤差の共分散行列P_predを計算
P_pred = F*P_est*F.' + Q;

% ── 2) 更新ステップ ──────────────────────────────────────────────────
% 観測ベクトルzを定義
z = [id_meas; iq_meas];

% 観測残差（イノベーション）yを計算
y = z - H*x_pred;

% 観測残差の共分散SとカルマンゲインKを計算
S = H*P_pred*H.' + R;
% K = P_pred*H.'/S;
K = P_pred * (H' / S);
% 状態xと共分散Pを更新
x_est = x_pred + K*y;
P_est = (eye(4) - K*H) * P_pred; % Joseph form for stability

% ── 出力 ───────────────────────────────────────────────────────────
dphi_d_est = x_est(3);
dphi_q_est = x_est(4);
id_est     = x_est(1);
iq_est     = x_est(2);
P_diag     = diag(P_est);
end

%% ----------------------------------------------------------------------------------
% 状態予測関数: 1ステップ先の状態を予測する
% ヤコビ行列の計算で再利用するため、J_invとphi_dotも出力する
%% ----------------------------------------------------------------------------------
function [x_next, J_inv, phi_dot] = ...
         f_discrete_delta_phi(x, u, Rs, Ts, ...
                              phi_d0, phi_q0, ...
                              Ldd, Ldq, Lqd, Lqq)

% --- 入力と状態の展開 ---
id    = x(1);    iq    = x(2);
dphid = x(3);    dphiq = x(4);
vd    = u(1);    vq    = u(2);    omega = u(3);

% --- 微分インダクタンス行列Jの作成 ---
% J = [∂φd/∂id, ∂φd/∂iq;
%      ∂φq/∂id, ∂φq/∂iq]
J = [Ldd, Ldq;
     Lqd, Lqq];
J_inv = inv(J); % 逆行列を計算（後の電流微分計算で使用）

% --- 実磁束の計算 ---
% 実磁束 = 基準マップ磁束 + EKFが推定した偏差Δφ
phi_d = phi_d0 + dphid;
phi_q = phi_q0 + dphiq;

% --- 磁束の時間微分の計算 ---
% 基本的な電圧方程式 v = R*i + dφ/dt + ω*J*φ を dφ/dt について解く
phi_dot_d = vd - Rs*id + omega*phi_q;
phi_dot_q = vq - Rs*iq - omega*phi_d;
phi_dot   = [phi_dot_d; phi_dot_q];

% --- 電流の時間微分の計算 ---
% dφ/dt = J * di/dt の関係から、di/dt = J_inv * dφ/dt
idiq_dot = J_inv * phi_dot;

% --- 状態の更新（オイラー法による離散化）---
id_next = id + Ts*idiq_dot(1);
iq_next = iq + Ts*idiq_dot(2);

% Δφはランダムウォークモデルなので、予測ステップでは変化しない
x_next = [id_next; iq_next; dphid; dphiq];
end

%% ----------------------------------------------------------------------------------
% 解析ヤコビ行列F = ∂f/∂x (4x4) を計算する
%% ----------------------------------------------------------------------------------
function F = calculate_F_delta_phi_analytic(x, u, Rs, Ts, ...
                                            J_inv, phi_dot, ...
                                            Ldd, Ldq, Lqd, Lqq, ...
                                            Hd,  Hq)

% --- 入力と状態の展開 ---
id = x(1);    iq = x(2); %#ok<NASGU> % NASGU: Not used, but needed for clarity
vd = u(1);    vq = u(2);    omega = u(3); %#ok<NASGU>

% --- 繰り返し使用する項の事前計算 ---
w = J_inv * phi_dot; % w = [did/dt; diq/dt]

% --- 状態変数(id, iq)に関するJの微分（テンソル）---
% dJ/did : Jをidで偏微分した2x2行列
dJ_did = [Hd(1,1), Hd(1,2);
          Hq(1,1), Hq(1,2)];
% dJ/diq : Jをiqで偏微分した2x2行列
dJ_diq = [Hd(2,1), Hd(2,2);
          Hq(2,1), Hq(2,2)];

% --- 状態変数に関するphi_dotの微分（2x4行列）---
% DP = [∂(phi_dot)/∂id, ∂(phi_dot)/∂iq, ∂(phi_dot)/∂Δφd, ∂(phi_dot)/∂Δφq]
DP = zeros(2,4);
DP(:,1) = [-Rs + omega*Lqd;       -omega*Ldd      ]; % ∂/∂id
DP(:,2) = [ omega*Lqq;            -Rs - omega*Ldq ]; % ∂/∂iq
DP(:,3) = [ 0;                    -omega          ]; % ∂/∂Δφd
DP(:,4) = [ omega;                0               ]; % ∂/∂Δφq

% --- ヤコビ行列Fの構築 ---
F = eye(4);

% % 状態ベクトルxの各要素(id, iq, Δφd, Δφq)でループ
% for idx = 1:4
%     % 状態変数に応じたdJ/dxを選択
%     switch idx
%         case 1, dJdx = dJ_did; % 状態がidの場合
%         case 2, dJdx = dJ_diq; % 状態がiqの場合
%         otherwise, dJdx = zeros(2,2);  % ΔφはJに影響しない
%     end

%     % dw/dx = d(J_inv * phi_dot)/dx を連鎖律で計算
%     % d(J_inv)/dx = -J_inv * (dJ/dx) * J_inv
%     % よって、dw/dx = -J_inv*(dJ/dx)*J_inv*phi_dot + J_inv*(d(phi_dot)/dx)
%     % w = J_inv*phi_dot なので、dw/dx = -J_inv*(dJ/dx)*w + J_inv*DP(:,idx)
%     dwdx = -J_inv*(dJdx*w) + J_inv*DP(:,idx);

%     % F = I + Ts * A の関係から、Fの対応する列を埋める
%     % A(1:2, idx) = dwdx
%     F(1,idx) = 1 + Ts*dwdx(1); % id_nextの偏微分
%     F(2,idx) =     Ts*dwdx(2); % iq_nextの偏微分
% end
% % Δφの行は [0 0 1 0] と [0 0 0 1] のままで、eye(4)で初期化済み


% % ── 前提：ここまでで計算済みの変数 ────────────────
% %   J_inv          : 2×2 逆インダクタンス行列
% %   w              : 2×1 = J_inv * phi_dot
% %   dJ_did, dJ_diq : 2×2   J の偏微分
% %   DP             : 2×4   ∂(phi_dot)/∂x  （先ほどと同じ定義）
% %   Ts             : サンプリング周期
% %   ※ x = [id iq Δφd Δφq] の順

% ── 1) di/dt の偏微分 dwdx を 4 本まとめて計算 ──
dwdx_id    = -J_inv*(dJ_did*w) + J_inv*DP(:,1);  % ∂w/∂id
dwdx_iq    = -J_inv*(dJ_diq*w) + J_inv*DP(:,2);  % ∂w/∂iq
dwdx_dphid =               J_inv*DP(:,3);        % ∂w/∂Δφd
dwdx_dphiq =               J_inv*DP(:,4);        % ∂w/∂Δφq

% ── 2) A 行列（連続時間の状態行列）をブロックで構築 ──
A = zeros(4);           % 4×4 のゼロ初期化
A(1:2, :) = [dwdx_id, ...
             dwdx_iq, ...
             dwdx_dphid, ...
             dwdx_dphiq];   % 上 2 行だけが非ゼロ

% ── 3) 離散時間ヤコビ F = I + Ts·A ───────────────
F = eye(4) + Ts * A;

end