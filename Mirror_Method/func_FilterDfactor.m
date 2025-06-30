function y = DFactorFilter_matlab(u)
% DFactorFilter_matlab
% 資料9.3節に基づく二相入力・二相出力D因子フィルタのMATLAB実装
%
%   Input:
%       u: 2x1の入力信号ベクトル [u1; u2]
%   Output:
%       y: 2x1の出力信号ベクトル [y1; y2]
%
%   内部で状態を保持するため、連続して呼び出すことでフィルタとして機能します。

% --- 設定 (関数内定義) ---
persistent x_p; % フィルタの内部状態を保持する永続変数

% パラメータ (固定値)
FS = 12000; % サンプリング周波数: 12kHz
A0 = 100.0; % F(s)の係数 a0
B0 = 100.0; % F(s)の係数 b0
B1 = 0.0;   % F(s)の係数 b1

% シフト周波数: 1kHz -> 2*pi*1000 rad/s
% 通過帯域の中心が1kHzになるように負値を設定
OMEGA0_SHIFT = -2 * pi * 1000.0;

% --- 永続変数の初期化 (初回実行時のみ) ---
if isempty(x_p)
    x_p = 0 + 0j;
end

% --- フィルタ処理 ---
Ts = 1.0 / FS;

% 1. 分解: 2相実数信号を複素数に変換
u_p = u(1) + 1j * u(2);

% 2. 複素フィルタリング (伝達関数と状態方程式に基づく離散化)
% s*x = -(a0 + j*omega0)*x + u_p
% これを離散化（後退差分）: (x_new - x_old)/Ts = -(a0+j*w0)*x_new + u_p
% (1 + (a0+j*w0)*Ts)*x_new = x_old + Ts*u_p
x_p_new = (x_p + Ts * u_p) / (1 + (A0 + 1j * OMEGA0_SHIFT) * Ts);

% 出力方程式 Y = (b0 - a0*b1)*x + b1*U
y_p = (B0 - A0 * B1) * x_p_new + B1 * u_p;

% 状態の更新
x_p = x_p_new;

% 3. 再構成: 複素数を出力2相実数信号に変換
y = [real(y_p); imag(y_p)];

end