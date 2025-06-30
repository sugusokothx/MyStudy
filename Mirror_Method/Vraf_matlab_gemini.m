function [y_alpha, y_beta] = vraf_codegen_fixed(u_alpha, u_beta, f_shift, Fs, seq_flag)
%#codegen

% 定数フィルタ係数（3次バターワースLPF例）
b = coder.const([0.0181, 0.0543, 0.0543, 0.0181]);
a = coder.const([1.0000, -1.7600, 1.1829, -0.2781]);

% 状態定義（フィルタ・位相）
persistent theta xR_hist xI_hist uR_hist uI_hist

if isempty(theta)
    theta = 0;
    xR_hist = zeros(3,1); % 過去出力 x[n-1], x[n-2], x[n-3]
    xI_hist = zeros(3,1);
    uR_hist = zeros(3,1); % 過去入力 u[n-1], u[n-2], u[n-3]
    uI_hist = zeros(3,1);
end

% ステップ毎の角度更新
delta_theta = 2 * pi * f_shift / Fs;
if seq_flag > 0
    theta = theta + delta_theta;
else
    theta = theta - delta_theta;
end

% 複素入力と復調（回転）
u_p = u_alpha + 1j * u_beta;
rot_demod = exp(-1j * theta);
u_tilde = u_p * rot_demod;
uR = real(u_tilde);
uI = imag(u_tilde);

% IIR フィルタを直接形で手動実装（例: 3次）
% y[n] = b0*u[n] + b1*u[n-1] + ... - a1*y[n-1] - a2*y[n-2] ...
xR = b(1)*uR + b(2)*uR_hist(1) + b(3)*uR_hist(2) + b(4)*uR_hist(3) ...
   - a(2)*xR_hist(1) - a(3)*xR_hist(2) - a(4)*xR_hist(3);
xI = b(1)*uI + b(2)*uI_hist(1) + b(3)*uI_hist(2) + b(4)*uI_hist(3) ...
   - a(2)*xI_hist(1) - a(3)*xI_hist(2) - a(4)*xI_hist(3);

% 状態更新
uR_hist = [uR; uR_hist(1:2)];
uI_hist = [uI; uI_hist(1:2)];
xR_hist = [xR; xR_hist(1:2)];
xI_hist = [xI; xI_hist(1:2)];

% 再変調（元の回転座標へ戻す）
x_tilde = xR + 1j * xI;
rot_remod = conj(rot_demod);
y_p = x_tilde * rot_remod;

% 出力分解
y_alpha = real(y_p);
y_beta = imag(y_p);

end
