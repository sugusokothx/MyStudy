以下では、すでに保有している

* **φ-map**（ `phi_d(id,iq)` , `phi_q(id,iq)` ）
* 対応する **軸ベクトル**（ `id_axis`、`iq_axis` ）

から――

1. **一次導関数マップ** `Ld_map(id,iq)=∂φd/∂id`, `Lq_map(id,iq)=∂φq/∂iq`
2. **二次導関数マップ** `Ldd,Ldq,Lqd,Lqq` （Hessian 行列要素）

を数値微分で生成して *.mat* に保存する **MATLAB スクリプト**を用意しました。
そのまま Simulink の 2-D Lookup Table に読み込める形（行列＋軸ベクトル）で出力します。

---

## 1. `build_L_maps.m` ― Ld/Lq（一次微分）生成

```matlab
function build_L_maps(varargin)
% build_L_maps
%  └─  first-order derivative maps:  Ld_map, Lq_map, Ldq_map, Lqd_map
%
% 必要ファイル
%   phi_map.mat  …  id_axis, iq_axis, phi_d, phi_q  を格納しておく
%
% 出力
%   L_map.mat    …  Ld_map, Lq_map, Ldq_map, Lqd_map  を保存
%
% オプション
%   'smooth',N   …  N>0 なら Savitzky–Golay (odd order 5, frame N) 平滑化
% ------------------------------------------------------------------------

% ----------------- ① φ-map 読み込み ----------------------------------
data = load('phi_map.mat');      % id_axis, iq_axis, phi_d, phi_q
id_axis = data.id_axis(:);       % enforce column vectors
iq_axis = data.iq_axis(:);

phi_d = data.phi_d;              % size: [numel(id_axis) , numel(iq_axis)]
phi_q = data.phi_q;

% ----------------- ② 数値微分（central difference） -------------------
[id_grid, iq_grid] = ndgrid(id_axis, iq_axis);         %#ok<ASGLU>

% MATLAB の gradient は「ラベル付け」すれば非等間隔グリッド対応
[dphi_d__did, dphi_d__diq] = gradient(phi_d, id_axis, iq_axis);
[dphi_q__did, dphi_q__diq] = gradient(phi_q, id_axis, iq_axis);

Ld_map  = dphi_d__did;   % ∂φd/∂id
Ldq_map = dphi_d__diq;   % ∂φd/∂iq
Lqd_map = dphi_q__did;   % ∂φq/∂id
Lq_map  = dphi_q__diq;   % ∂φq/∂iq

% ----------------- ③ オプション平滑化 -------------------------------
if any(strcmpi(varargin,'smooth'))
    N = varargin{find(strcmpi(varargin,'smooth'))+1};
    if ~isempty(N) && N>0
        sg = @(M) sgolayfilt(M,5, N);   % 5th-order polynomial
        Ld_map  = sg(Ld_map);
        Lq_map  = sg(Lq_map);
        Ldq_map = sg(Ldq_map);
        Lqd_map = sg(Lqd_map);
    end
end

% ----------------- ④ 保存 --------------------------------------------
save('L_map.mat', 'id_axis','iq_axis', ...
                  'Ld_map','Lq_map','Ldq_map','Lqd_map');
fprintf('[build_L_maps]  ->  L_map.mat written\n');
end
```

> **使い方例**
>
> ```matlab
> build_L_maps('smooth',11);   % 11-point S-G 平滑化付き
> ```

---

## 2. `build_L_hessian.m` ― 2次導関数（Hessian）生成

```matlab
function build_L_hessian(varargin)
% build_L_hessian
%  └─ second-order derivative maps (Hessian elements)
%      Ldd = ∂²φd/∂id²,   Ldq = ∂²φd/∂id∂iq
%      Lqd = ∂²φq/∂id∂iq, Lqq = ∂²φq/∂iq²
%
% 依存: L_map.mat （一次導関数マップ）  or  phi_map.mat から直接計算
% 出力: L_hess.mat
% ----------------------------------------------------------------------

% ----------------- ① 元データの確保 ---------------------------------
if exist('L_map.mat','file')   % すでに一次微分があれば再利用
    S = load('L_map.mat');
    id_axis = S.id_axis; iq_axis = S.iq_axis;
    dphi_d__did = S.Ld_map;
    dphi_d__diq = S.Ldq_map;
    dphi_q__did = S.Lqd_map;
    dphi_q__diq = S.Lq_map;
else                           % なければ一次導関数から生成
    build_L_maps(varargin{:});
    S = load('L_map.mat');
    id_axis = S.id_axis; iq_axis = S.iq_axis;
    dphi_d__did = S.Ld_map;
    dphi_d__diq = S.Ldq_map;
    dphi_q__did = S.Lqd_map;
    dphi_q__diq = S.Lq_map;
end

% ----------------- ② 二次微分 ----------------------------------------
[Ldd, ~         ] = gradient(dphi_d__did, id_axis, iq_axis); % ∂/∂id
[~,    Ldq       ] = gradient(dphi_d__diq, id_axis, iq_axis); % ∂/∂iq
[Lqd, ~          ] = gradient(dphi_q__did, id_axis, iq_axis);
[~,    Lqq       ] = gradient(dphi_q__diq, id_axis, iq_axis);

% ----------------- ③ オプション平滑化 --------------------------------
if any(strcmpi(varargin,'smooth'))
    N = varargin{find(strcmpi(varargin,'smooth'))+1};
    if ~isempty(N) && N>0
        sg = @(M) sgolayfilt(M,5,N);
        Ldd = sg(Ldd);  Ldq = sg(Ldq);
        Lqd = sg(Lqd);  Lqq = sg(Lqq);
    end
end

% ----------------- ④ 保存 --------------------------------------------
save('L_hess.mat','id_axis','iq_axis','Ldd','Ldq','Lqd','Lqq');
fprintf('[build_L_hessian]  ->  L_hess.mat written\n');
end
```

---

### ✅ これで出来上がるファイル

| ファイル          | 格納変数                                                           | 用途                        |
| ------------- | -------------------------------------------------------------- | ------------------------- |
| `phi_map.mat` | `id_axis`, `iq_axis`, `phi_d`, `phi_q`                         | **既存**：実測・FEM 等から得た φ-map |
| `L_map.mat`   | `id_axis`, `iq_axis`, `Ld_map`, `Lq_map`, `Ldq_map`, `Lqd_map` | EKF の一次導関数 LUT            |
| `L_hess.mat`  | `id_axis`, `iq_axis`, `Ldd`, `Ldq`, `Lqd`, `Lqq`               | EKF（STEP 2）の Hessian LUT  |

---

## 3. Simulink への組み込みポイント

1. **Pre-LoadFcn** 等で

   ```matlab
   load L_map.mat;   load L_hess.mat;
   ```

   を実行し、ワークスペースに行列と軸を展開。

2. `2-D Lookup Table` ブロックに

   * *Table data* → `Ld_map` （など）
   * *Breakpoints 1* → `id_axis`, *Breakpoints 2* → `iq_axis`
     を設定するだけで完了です。

3. EKF ブロック（前回答の `ekf_delta_phi_estimator`）には

   ```matlab
   Ld_map(id_idx,iq_idx)  などを実時間で参照 → Mux → EKF
   ```

   という信号配線にしてください。

---

### 補足 : 精度と安定性

| 項目             | 推奨設定                                        |
| -------------- | ------------------------------------------- |
| グリッド間隔         | `Δid`, `Δiq` ともに **5–10 A** 程度以下（非線形領域は細かく） |
| `gradient` の誤差 | グリッドが粗い場合やノイズが多い場合は `smooth` オプション推奨        |
| 端点処理           | `gradient` は両端で一次精度になるため、境界外挿しない設計が安全       |

これで EKF STEP-1/STEP-2 の**すべてのマップ**が自前で生成でき、Simulink へそのまま渡せます。
次フェーズ（STEP-2 の性能検証や EKF パラメータのチューニング）が必要になったら、また声を掛けてください！
