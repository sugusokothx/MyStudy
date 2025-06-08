function y = pmsm_dq_model(u)
%#codegen
% ───────────────────────────────────────────────────────
% PMSM dq-axis model (discrete, Euler integration)
%
% Inputs
%   u(1:2) : v_dq  = [v_d, v_q]  [V]
%   u(3)   : omega = electrical angular speed [rad/s]
%
% Outputs
%   y(1:2) : i_dq  = [i_d, i_q]  [A]
%   y(3)   : Te    = electromagnetic torque [N·m]
% ───────────────────────────────────────────────────────

%% ── 0)  パラメータ & ルックアップ準備 ─────────────
%   （ワークスペースから呼び出す）
coder.extrinsic('evalin');
Rs    = evalin('base', 'Rs');
Pn    = evalin('base', 'Pn');
Ts    = evalin('base', 'Ts');

psi_d_axis = evalin('base', 'psi_d_axis');
psi_q_axis = evalin('base', 'psi_q_axis');
IdTable    = evalin('base', 'IdTable');
IqTable    = evalin('base', 'IqTable');

persistent psi_d psi_q     % ステータフラックス [Wb]
if isempty(psi_d)
    psi_d = 0; psi_q = 0;
end

%% ── 1)  入力展開 ───────────────────────────────
v_d   = u(1);
v_q   = u(2);
omega = u(3);

%% ── 2)  現在フラックス → 電流（ID / IQ マップ） ───
i_d = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IdTable);
i_q = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IqTable);

%% ── 3)  フラックス微分 & オイラー積分 ────────────
%   dψ/dt = v_dq – R_s·i_dq + ω·J·ψ
dpsi_d = v_d - Rs*i_d - omega*psi_q;   % note the sign (J matrix)
dpsi_q = v_q - Rs*i_q + omega*psi_d;

psi_d = psi_d + Ts * dpsi_d;
psi_q = psi_q + Ts * dpsi_q;

%% ── 4)  更新後フラックスで再計算（オプション） ─────
i_d = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IdTable);
i_q = table2D(psi_d, psi_q, psi_d_axis, psi_q_axis, IqTable);

%% ── 5)  電磁トルク ─────────────────────────────
Te = Pn * (psi_d*i_q - psi_q*i_d);

%% ── 6)  出力 ───────────────────────────────────
y = [i_d, i_q, Te];

end   % ―― main function END ――

% ======================================================
%  2-D Lookup helper  (bilinear interpolation, no extrap)
% ======================================================
function val = table2D(x, y, xAxis, yAxis, Tbl)
% Linear-in-both-axes interpolation (size checks省略)
%   xAxis: 1×Nx, yAxis: 1×Ny, Tbl: Ny×Nx (MATLAB default)

% --- find surrounding indices
ix = find(xAxis <= x, 1, 'last');
iy = find(yAxis <= y, 1, 'last');
% saturate at boundaries
ix = max(min(ix, numel(xAxis)-1),1);
iy = max(min(iy, numel(yAxis)-1),1);

% --- fractional distance
dx = (x - xAxis(ix)) / (xAxis(ix+1) - xAxis(ix));
dy = (y - yAxis(iy)) / (yAxis(iy+1) - yAxis(iy));

% --- four-point bilinear blend
v11 = Tbl(iy    , ix    );
v21 = Tbl(iy    , ix + 1);
v12 = Tbl(iy + 1, ix    );
v22 = Tbl(iy + 1, ix + 1);

val = (1-dx)*(1-dy)*v11 + dx*(1-dy)*v21 + ...
      (1-dx)*dy*v12   + dx*dy*v22;
end
