% function signature: Ld, Lqの代わりにΔφd, Δφqを出力
function [delta_phi_d_est, delta_phi_q_est, P_diag, Id_est, Iq_est] = ekf_delta_phi_estimator(id_meas, iq_meas, vd, vq, omega)
% EKF for Delta Phi Estimation

% persistent変数の変更: x_estは4次元に
persistent x_est P_est

% --- 固定パラメータ ---
Rs = 30/1000; % 固定子抵抗
Ts = 1/5000;  % サンプリング時間

% --- チューニングパラメータ (ノイズ共分散行列) ---
% Q: 状態 [id, iq, Δφd, Δφq] のプロセスノイズ
% Δφの変動は遅いため、対応するQ(3,3), Q(4,4)は非常に小さい値に設定
Q = diag([1e-4, 1e-4, 1e-9, 1e-9]); 

% R: 観測 [id, iq] のノイズ
R = diag([1e-3, 1e-3]);

% --- 観測モデル ---
% H: 状態xから観測zを写す行列. z = H*x
H = [1 0 0 0;
     0 1 0 0];

% --- 初期化 ---
if isempty(x_est)
    % 状態ベクトル: [id; iq; Δφd; Δφq]
    % 初期偏差Δφはゼロと仮定
    x_est = [0; 0; 0; 0];
    
    % 初期共分散行列P: Δφの初期不確かさを大きく設定
    P_est = diag([0.01, 0.01, 1e-2, 1e-2]);
end

% --- 入力と観測の整理 ---
z = [id_meas; iq_meas]; % 観測ベクトル (実測電流)
u = [vd; vq; omega];   % 入力ベクトル

% =================================================================
% EKFアルゴリズム
% =================================================================

% --- 1. 予測 (Prediction) ---

% 予測ステップに必要なマップ値を取得 (この部分は別途実装が必要)
% [phi_d_map, phi_q_map] = lookup_phi_map(x_est(1), x_est(2));
% [Ld_map, Lq_map] = lookup_L_map(x_est(1), x_est(2)); % モデルで使う等価L
% [Ldd, Ldq, Lqd, Lqq] = lookup_diff_L_map(x_est(1), x_est(2)); % ヤコビ計算で使う微分L

% ※上記マップ参照部分は仮の値でプレースホルダを置きます
phi_d_map = 0.1; phi_q_map = 0;
Ld_map = 0.012; Lq_map = 0.012;
Ldd = 0.012; Ldq = 0; Lqd = 0; Lqq = 0.012;


% 状態方程式 f(x,u) に基づいて状態を予測
x_pred = f_discrete_delta_phi(x_est, u, Rs, Ts, phi_d_map, phi_q_map, Ld_map, Lq_map);

% ヤコビ行列 F の計算
F = calculate_F_delta_phi(x_est, u, Rs, Ts, Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

% 予測誤差の共分散行列 P_pred の計算
P_pred = F * P_est * F' + Q;


% --- 2. 更新 (Update) ---

% 観測残差 y (イノベーション)
y = z - H * x_pred;

% カルマンゲイン K の計算
S = H * P_pred * H' + R;
K = P_pred * H' / S; % inv(S)の代わりに右除算

% 状態ベクトル x_est の更新
x_est = x_pred + K * y;

% 推定誤差の共分散行列 P_est の更新
P_est = (eye(4) - K * H) * P_pred;


% --- 出力 ---
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
P_diag = diag(P_est);
Id_est = x_est(1);
Iq_est = x_est(2);

end


% =================================================================
% 下位関数 (新規作成・修正)
% =================================================================

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
    A(2,3) = -omega / Lq_map;

    % 離散化: F = I + A*Ts (1次近似)
    F = eye(4) + A * Ts;
end