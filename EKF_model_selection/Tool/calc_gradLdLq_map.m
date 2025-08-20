%% ========================================================================
%  実行サンプルセクション
% =========================================================================

% --- 1. サンプルの磁束マップを作成 ---
% (実際には、この部分をFEM解析や実測で得たデータに置き換えてください)

% 電流の格子ベクトルを定義
id_vec = -200:20:0;     % d軸電流ベクトル [A]
iq_vec = 0:20:300;      % q軸電流ベクトル [A]

% グリッドを作成
[Id_grid, Iq_grid] = meshgrid(id_vec, iq_vec);

% IPMモータの簡易的な物理モデルで磁束マップを生成
psi_f = 0.1;    % 永久磁石磁束 [Wb]
Ld0 = 0.0003;   % d軸インダクタンス [H]
Lq0 = 0.0005;   % q軸インダクタンス [H]

% 簡易的な飽和を考慮 (実際はより複雑なLUTになります)
phid_map = psi_f + Ld0 * Id_grid .* (1 - 0.2 * (Id_grid/max(abs(id_vec))).^2);
phiq_map = Lq0 * Iq_grid .* (1 - 0.3 * (Iq_grid/max(iq_vec)).^2);


% --- 2. 微分インダクタンスマップを計算 ---
% 作成した関数を呼び出します
[Ldd, Ldq, Lqd, Lqq] = create_diff_L_maps(id_vec, iq_vec, phid_map, phiq_map);


% --- 3. 結果を可視化して確認 ---
figure('Name', 'Differential Inductance Map Creation');

% 元の磁束マップをプロット
subplot(2, 2, 1);
surf(Id_grid, Iq_grid, phid_map);
title('\phi_d Map (Original)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('\phi_d [Wb]');

% 計算されたLddマップをプロット
subplot(2, 2, 2);
surf(Id_grid, Iq_grid, Ldd);
title('L_{dd} Map (Calculated)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('L_{dd} [H]');

% 元の磁束マップをプロット
subplot(2, 2, 3);
surf(Id_grid, Iq_grid, phiq_map);
title('\phi_q Map (Original)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('\phi_q [Wb]');

% 計算されたLqqマップをプロット
subplot(2, 2, 4);
surf(Id_grid, Iq_grid, Lqq);
title('L_{qq} Map (Calculated)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('L_{qq} [H]');

%% ========================================================================
%  微分インダクタンスマップ作成関数
% =========================================================================
function [Ldd, Ldq, Lqd, Lqq] = create_diff_L_maps(id_vec, iq_vec, phid_map, phiq_map)
% CREATE_DIFF_L_MAPS は、順方向の磁束マップから微分インダクタンスマップを計算します。
%
% Inputs:
%   id_vec      - d軸電流のベクトル (1 x M)
%   iq_vec      - q軸電流のベクトル (1 x N)
%   phid_map    - φdの2次元マップ (N x M)
%   phiq_map    - φqの2次元マップ (N x M)
%
% Outputs:
%   Ldd, Ldq, Lqd, Lqq - 計算された微分インダクタンスの2次元マップ (N x M)

    % gradient関数は、[dF/dx, dF/dy] の順で勾配を返します。
    % 今回のマップの次元は (iq, id) なので、
    % y方向が iq, x方向が id に対応します。
    %
    % したがって、[dF/did, dF/diq] の順で出力されます。

    % 1. φdマップからLddとLdqを計算
    %    Ldd = d(φd)/d(id),  Ldq = d(φd)/d(iq)
    [Ldd, Ldq] = gradient(phid_map, id_vec, iq_vec);

    % 2. φqマップからLqdとLqqを計算
    %    Lqd = d(φq)/d(id),  Lqq = d(φq)/d(iq)
    [Lqd, Lqq] = gradient(phiq_map, id_vec, iq_vec);

end