% --- 0) データ (ID, IQ, φd, φq) を読み込み済みと仮定 ----------
X  = [ID(:), IQ(:)];         % N×2
Yd = phi_d_map(:);           % N×1
Yq = phi_q_map(:);

% --- 1) 基底行列＆偏微分行列を取得 ----------------------------
n = 3;
[P, dPid, dPiq, d2Pid2, d2Piq2] = polybasis2d(X, n);

% --- 2) エネルギー関数 W(id,iq) の係数を最小二乗で推定 --------
A = [dPid; dPiq];            % [∂W/∂id; ∂W/∂iq] = φ_target
b = [Yd;   Yq ];
coef = A \ b;                % M×1

% --- 3) 滑らかな φd, φq, Ld, Lq を生成 -------------------------
phi_d_fit = reshape(dPid*coef,  size(ID));
phi_q_fit = reshape(dPiq*coef,  size(ID));
Ld_map    = reshape(d2Pid2*coef, size(ID));  % ∂²W/∂id²
Lq_map    = reshape(d2Piq2*coef, size(ID));  % ∂²W/∂iq²

% --- 4) 可視化してチェック ------------------------------------
figure; surf(ID, IQ, Ld_map*1e3, 'EdgeColor','none', 'FaceAlpha',0.7);
xlabel('i_d [A]'); ylabel('i_q [A]'); zlabel('L_d [mH]');
title('Smooth L_d via Energy-Function Fit'); colorbar; view(40,30);



function [P, dPid, dPiq, d2Pid2, d2Piq2, exps] = polybasis2d(X, n)
%POLYBASIS2D  2-D polynomial basis up to total order n.
%
% [P, dPid, dPiq, d2Pid2, d2Piq2, exps] = polybasis2d(X, n)
%
% Inputs
%   X   : (N × 2) matrix, columns are id and iq
%   n   : maximum total degree (non-negative integer)
%
% Outputs
%   P        : (N × M)       basis matrix         φ_k(id,iq)
%   dPid     : (N × M)       ∂φ_k/∂id
%   dPiq     : (N × M)       ∂φ_k/∂iq
%   d2Pid2   : (N × M)       ∂²φ_k/∂id²
%   d2Piq2   : (N × M)       ∂²φ_k/∂iq²
%   exps     : (M × 2) int   [p  q] exponents   (id^p iq^q)
%
% Example
%   [P, dPid, dPiq] = polybasis2d([0 0; 1 2; -1 3], 3);
%   % → third-order polynomial basis evaluated at 3 points
%
% Author: ChatGPT  (July-2025)

% -------------------------------------------------------------------------
arguments
    X   (:,2) double
    n   (1,1) {mustBeInteger, mustBeNonnegative}
end
N = size(X,1);
id = X(:,1);
iq = X(:,2);

% ---- 1) exponent list [p,q] with p+q <= n --------------------------------
exps = [];
for p = 0:n
    for q = 0:(n-p)
        exps = [exps; p, q]; %#ok<AGROW>
    end
end
M = size(exps,1);

% ---- 2) preallocate output matrices --------------------------------------
P        = zeros(N, M);
dPid     = zeros(N, M);
dPiq     = zeros(N, M);
d2Pid2   = zeros(N, M);
d2Piq2   = zeros(N, M);

% ---- 3) evaluate monomials and derivatives -------------------------------
for k = 1:M
    p = exps(k,1);
    q = exps(k,2);

    % monomial
    monom = (id.^p) .* (iq.^q);
    P(:,k) = monom;

    % 1st derivatives
    if p > 0
        dPid(:,k) = p * (id.^(p-1)) .* (iq.^q);
        if p > 1
            d2Pid2(:,k) = p*(p-1) * (id.^(p-2)) .* (iq.^q);
        end
    end
    if q > 0
        dPiq(:,k) = q * (id.^p) .* (iq.^(q-1));
        if q > 1
            d2Piq2(:,k) = q*(q-1) * (id.^p) .* (iq.^(q-2));
        end
    end
end
end

