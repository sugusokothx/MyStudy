function [vd_corr, vq_corr, d_vd, d_vq] = deadTimeCompDQ( ...
        vd_cmd, vq_cmd,  ...  % dq 指令電圧 [V]
        id, iq,             ...  % dq 電流 [A]
        theta_e,            ...  % 電気角 [rad]
        Vdc, t_dead, Tsw,   ...  % DC-Link[V], デッドタイム[s], スイッチ周期[s]
        I_eps)                  % 零交差抑制閾値[A]
%#codegen
%---------------------------------------------------------------
% Dead-time compensation in dq frame (formula 1-B)
%   Δvd = -(2/3)*Vtd*(sgn(id)*cosθ - sgn(iq)*sinθ)
%   Δvq = -(2/3)*Vtd*(sgn(id)*sinθ + sgn(iq)*cosθ)
%---------------------------------------------------------------

% ── 0. 符号保持用の persistent 変数 ──────────────────────────
persistent id_prev iq_prev
if isempty(id_prev); id_prev = 0; end
if isempty(iq_prev); iq_prev = 0; end

% ── 1. 定数計算 ────────────────────────────────
Vtd = Vdc * t_dead / Tsw;   % デッドタイム電圧 [V]
k   = -2/3 * Vtd;           % 係数

% ── 2. 符号関数（零交差ヒステリシス込み）───────────
if abs(id) < I_eps
    sgn_id = sign(id_prev);
else
    sgn_id = sign(id);
end

if abs(iq) < I_eps
    sgn_iq = sign(iq_prev);
else
    sgn_iq = sign(iq);
end

% ── 3. 補正電圧 Δvd, Δvq 計算 ───────────────────
c = cos(theta_e);
s = sin(theta_e);

d_vd = k * ( sgn_id * c - sgn_iq * s );
d_vq = k * ( sgn_id * s + sgn_iq * c );

% ── 4. 指令電圧へ加算 ──────────────────────────
vd_corr = vd_cmd + d_vd;
vq_corr = vq_cmd + d_vq;

% ── 5. 符号を次回へ保持 ─────────────────────────
id_prev = sgn_id;
iq_prev = sgn_iq;
end
