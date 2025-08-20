sigma = 1;                          % 1~2 グリッド分が目安
phi_d_s = imgaussfilt(phi_d_map, sigma, ...
                      'Padding','replicate');
phi_q_s = imgaussfilt(phi_q_map, sigma, 'Padding','replicate');

[~, Ld_map] = gradient(phi_d_s, d_id, d_iq);
[~, Lq_map] = gradient(phi_q_s, d_id, d_iq);



% φd, φq を Stack → ベクトル化
X = [id(:), iq(:)];
Yd = phi_d_map(:);    Yq = phi_q_map(:);

% 3 次多項式基底 (1, id, iq, id^2, id*iq, iq^2, ...)
P = polybasis2d(X, 3);              % † 自作関数 or FileExchange

% W(id,iq) を最小二乗で推定
% φd = dW/did, φq = dW/diq の線形連立
A = [dP_did; dP_diq];               % 偏微分行列
b = [Yd;     Yq ];
coef = A\b;                         % W の係数

% >>> 滑らかな φ & L を解析計算 <<<
phi_d_fit = reshape(dP_did*coef,  size(ID));
phi_q_fit = reshape(dP_diq*coef,  size(ID));
Ld_map    = reshape(d2P_did2*coef, size(ID));  % ∂²W/∂id²
Lq_map    = reshape(d2P_diq2*coef, size(ID));  % ∂²W/∂iq²



