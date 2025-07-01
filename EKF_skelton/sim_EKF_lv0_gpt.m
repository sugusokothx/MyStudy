function [psi_f_est, deltaLd_est, deltaLq_est] = IPM_EKF_Block(vd, vq, omega_e, id_meas, iq_meas)
% IPM_EKF_Block   拡張カルマンフィルタ本体
%   入力:
%     vd, vq       … dq 軸電圧指令
%     omega_e      … 電気角速度
%     id_meas, iq_meas … dq 軸電流センサ測定値
%   出力:
%     psi_f_est    … 推定磁束
%     deltaLd_est  … Ld の偏差推定値
%     deltaLq_est  … Lq の偏差推定値

  persistent x_prev P_prev params_initialized params
  if isempty(params_initialized)
    %--- 初期化 (公称値, Q, R, P0, x0 などを params 構造体へ) ---
    params = struct( ...
      'Ld_nom', Ld_nom, 'Lq_nom', Lq_nom, ...
      'Rs', Rs, 'Ts', Ts, ...
      'Q', diag([Q_id, Q_iq, Q_psi, Q_dLd, Q_dLq]), ...
      'R', diag([R_id, R_iq]) );
    x_prev = [0; 0; psi0; 0; 0];      % [id; iq; psi_f; ΔLd; ΔLq]
    P_prev = eye(5) * P0;
    params_initialized = true;
  end

  %--- 1) 予測ステップ ---
  [x_pred, P_pred] = IPM_EKF_predictStep(x_prev, P_prev, vd, vq, omega_e, params);

  %--- 2) 更新ステップ ---
  z = [id_meas; iq_meas];
  [x_upd, P_upd] = IPM_EKF_updateStep(x_pred, P_pred, z, params);

  %--- 結果出力 & 次ステップへ保持 ---
  x_prev = x_upd;
  P_prev = P_upd;
  psi_f_est   = x_upd(3);
  deltaLd_est = x_upd(4);
  deltaLq_est = x_upd(5);
end


%% 補助関数：予測ステップ
function [x_pred, P_pred] = IPM_EKF_predictStep(x, P, vd, vq, omega_e, prm)


  % 1) 状態遷移 f(x,u)
  Ld = prm.Ld_nom + x(4);
  Lq = prm.Lq_nom + x(5);
  f = [ ...
    ( -prm.Rs*x(1) + omega_e*Lq*x(2) + vd ) / Ld; 
    ( -prm.Rs*x(2) - omega_e*Ld*x(1) - omega_e*x(3) + vq ) / Lq;
    0; 0; 0 ];
  x_pred = x + prm.Ts * f;

  if 0
  % 2) ヤコビアン F の計算
  F = IPM_EKF_computeJacobian(x, omega_e, prm);
  % 3) 共分散予測
  P_pred = F * P * F' + prm.Q;
  else
  u_vec = [vd; vq; omega_e];
  F_c   = IPM_EKF_computeJacobianNum(x, u_vec, prm);  % 連続→離散化込み
  P_pred = F_c * P * F_c' + prm.Q;
  end

end


%% 補助関数：更新ステップ
function [x_upd, P_upd] = IPM_EKF_updateStep(x_pred, P_pred, z, prm)
  % 観測モデル y = H x + noise
  H = [1 0 0 0 0;
       0 1 0 0 0];
  S = H * P_pred * H' + prm.R;
  K = P_pred * H' / S;

  % イノベーション
  y_pred = H * x_pred;
  x_upd  = x_pred + K * (z - y_pred);

  % 共分散更新
  P_upd = (eye(numel(x_pred)) - K*H) * P_pred;
end


%% 補助関数：状態ヤコビアン F を計算
function F = IPM_EKF_computeJacobian(x, omega_e, prm)
  Ld = prm.Ld_nom + x(4);
  Lq = prm.Lq_nom + x(5);
  Ts = prm.Ts;
  Rs = prm.Rs;
  % 部分微分を手計算 or Symbolic Toolbox で求めた式を埋め込む
  F = eye(5);
  F(1,1) = 1 - Rs*Ts/Ld;
  F(1,2) = omega_e*Ts*Lq/Ld;
  F(1,4) = -Ts*( -Rs*x(1) + omega_e*Lq*x(2) + prm.vd ) / Ld^2;
  % …その他の要素も同様に…
  % F(2,*), F(3,*) は [0 0 1 0 0], … など
end


%% 補助関数：状態ヤコビアン F を「数値微分」で計算
function F = IPM_EKF_computeJacobianNum(x, u_vec, prm)
%  x      : 現在の状態ベクトル (5×1)
%  u_vec  : [vd; vq; omega_e] (3×1)
%  prm    : 構造体（Ld_nom, Lq_nom, Rs, Ts など）
%
%  出力 F : 5×5 数値ヤコビアン  F ≈ ∂f/∂x |_(x)

    n = numel(x);
    F = zeros(n);

    % シミュレーションで扱う摂動量（機械的に 1e-6 倍のスケール）
    eps_rel = 1e-6;

    % 基準の状態遷移 f(x) を計算
    f0 = f_dx(x, u_vec, prm);   % 自前の状態方程式を呼び出す（下に定義）

    % 各状態成分を ε だけ正負にずらして中央差分
    for i = 1:n
        dx      = zeros(n,1);
        eps_abs = eps_rel * max(1, abs(x(i)));  % スケール依存で調整
        dx(i)   = eps_abs;

        f_plus  = f_dx(x + dx, u_vec, prm);
        f_minus = f_dx(x - dx, u_vec, prm);

        % 中央差分 (f+ − f−) / (2ε)
        F(:,i) = (f_plus - f_minus) / (2 * eps_abs);
    end

    % 離散化：  F_d = I + Ts * F_c
    F = eye(n) + prm.Ts * F;
end


%% ローカル関数：連続時間の状態方程式 f_c(x,u)
function f = f_dx(x, u, prm)
    % 分解
    id = x(1);  iq = x(2);  psi = x(3);
    dLd = x(4); dLq = x(5);
    vd  = u(1); vq = u(2);  omega_e = u(3);

    Ld = prm.Ld_nom + dLd;
    Lq = prm.Lq_nom + dLq;

    % 連続時間モデル
    f = zeros(5,1);
    f(1) = (-prm.Rs*id + omega_e*Lq*iq + vd) / Ld;
    f(2) = (-prm.Rs*iq - omega_e*Ld*id - omega_e*psi + vq) / Lq;
    % ランダムウォーク
    f(3) = 0;        % dψ_f/dt
    f(4) = 0;        % d(ΔLd)/dt
    f(5) = 0;        % d(ΔLq)/dt
end
