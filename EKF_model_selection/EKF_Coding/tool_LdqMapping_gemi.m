%% ========================================================================
%  実行サンプルセクション
% =========================================================================

% --- 1. 順方向の磁束マップを準備 ---
% (実際には、この部分をFEM解析や実測で得たご自身のデータに置き換えてください)

% 電流の格子ベクトルを定義
id_vec = -200:20:0;     % d軸電流ベクトル [A]
iq_vec = 0:20:300;      % q軸電流ベクトル [A]

% グリッドを作成
[Id_grid, Iq_grid] = meshgrid(id_vec, iq_vec);

% IPMモータの簡易的な物理モデルで磁束マップを生成
psi_f = 0.1;    % 永久磁石磁束 [Wb]
Ld0 = 0.0003;   % d軸インダクタンス [H]
Lq0 = 0.0005;   % q軸インダクタンス [H]
% 簡易的な飽和を考慮
phid_map = psi_f + Ld0 * Id_grid .* (1 - 0.2 * (Id_grid/max(abs(id_vec))).^2);
phiq_map = Lq0 * Iq_grid .* (1 - 0.3 * (Iq_grid/max(iq_vec)).^2);


% --- 2. 微分マップを計算 ---
% 作成した関数を呼び出し、すべてのマップを構造体として受け取る
derivative_maps = create_all_derivative_maps(id_vec, iq_vec, phid_map, phiq_map);

% これで、`derivative_maps.Ldd` のように各マップにアクセスできます。


% --- 3. 結果を可視化して確認 ---
figure('Name', 'Derivative Map Generation');

% Lddマップ (1階微分)
subplot(1, 3, 1);
surf(Id_grid, Iq_grid, derivative_maps.Ldd);
title('L_{dd} Map (1st derivative)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('L_{dd} [H]');
view(-30, 25);

% Lqqマップ (1階微分)
subplot(1, 3, 2);
surf(Id_grid, Iq_grid, derivative_maps.Lqq);
title('L_{qq} Map (1st derivative)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('L_{qq} [H]');
view(-30, 25);

% ヘシアンの1成分 (2階微分)
subplot(1, 3, 3);
surf(Id_grid, Iq_grid, derivative_maps.phid_id_id);
title('\partial^2\phi_d / \partiali_d^2 Map (2nd derivative)');
xlabel('id [A]'); ylabel('iq [A]'); zlabel('[\cdot]');
view(-30, 25);


%% ========================================================================
%  微分マップ作成関数（1階・2階をまとめて計算）
% =========================================================================
function maps = create_all_derivative_maps(id_vec, iq_vec, phid_map, phiq_map)
% CREATE_ALL_DERIVATIVE_MAPS は、順方向の磁束マップから
% 1階および2階のすべての微分マップを計算します。
%
% Inputs:
%   id_vec      - d軸電流のベクトル (1 x M)
%   iq_vec      - q軸電流のベクトル (1 x N)
%   phid_map    - φdの2次元マップ (N x M)
%   phiq_map    - φqの2次元マップ (N x M)
%
% Outputs:
%   maps        - 計算されたすべての微分マップを含む構造体

    % --- 1. 1階微分マップの計算 ---
    % Ldd = d(φd)/d(id),  Ldq = d(φd)/d(iq)
    [Ldd, Ldq] = gradient(phid_map, id_vec, iq_vec);
    % Lqd = d(φq)/d(id),  Lqq = d(φq)/d(iq)
    [Lqd, Lqq] = gradient(phiq_map, id_vec, iq_vec);

    % --- 2. 2階微分マップ（ヘシアン）の計算 ---
    % 1階微分マップをさらに微分する
    [phid_id_id, phid_id_iq] = gradient(Ldd, id_vec, iq_vec);
    [~, phid_iq_iq]         = gradient(Ldq, id_vec, iq_vec);

    [phiq_id_id, phiq_id_iq] = gradient(Lqd, id_vec, iq_vec);
    [~, phiq_iq_iq]         = gradient(Lqq, id_vec, iq_vec);

    % --- 3. 結果を構造体に格納 ---
    maps = struct();
    % 1st derivatives
    maps.Ldd = Ldd;
    maps.Ldq = Ldq;
    maps.Lqd = Lqd;
    maps.Lqq = Lqq;
    % 2nd derivatives (Hessian components for phi_d)
    maps.phid_id_id = phid_id_id;
    maps.phid_id_iq = phid_id_iq;
    maps.phid_iq_iq = phid_iq_iq;
    % 2nd derivatives (Hessian components for phi_q)
    maps.phiq_id_id = phiq_id_id;
    maps.phiq_id_iq = phiq_id_iq;
    maps.phiq_iq_iq = phiq_iq_iq;
end