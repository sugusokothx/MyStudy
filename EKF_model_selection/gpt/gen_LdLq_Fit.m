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
