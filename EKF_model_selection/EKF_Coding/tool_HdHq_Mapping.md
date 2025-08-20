## Hd, Hq とは？ — 定義をもう一度だけ整理

* **φd(id, iq)**, **φq(id, iq)** : 基準磁束マップ
* 1 次偏導関数

  $$
    J=\begin{bmatrix}
      L_{dd}&L_{dq}\\
      L_{qd}&L_{qq}
    \end{bmatrix}
    =
    \begin{bmatrix}
      \partial\phi_d/\partial id & \partial\phi_d/\partial iq\\[4pt]
      \partial\phi_q/\partial id & \partial\phi_q/\partial iq
    \end{bmatrix}
  $$
* **Hd** と **Hq** はそれぞれ φd, φq の **2 次偏導（ヘシアン）**

  $$
  H_d=\begin{bmatrix}
    \dfrac{\partial^2\phi_d}{\partial id^2} & \dfrac{\partial^2\phi_d}{\partial id\partial iq}\\[10pt]
    \dfrac{\partial^2\phi_d}{\partial iq\partial id} & \dfrac{\partial^2\phi_d}{\partial iq^2}
  \end{bmatrix},
  \quad
  H_q=\begin{bmatrix}
    \dfrac{\partial^2\phi_q}{\partial id^2} & \dfrac{\partial^2\phi_q}{\partial id\partial iq}\\[10pt]
    \dfrac{\partial^2\phi_q}{\partial iq\partial id} & \dfrac{\partial^2\phi_q}{\partial iq^2}
  \end{bmatrix}
  $$

---

## 1️⃣ オフラインで **数値微分**して LUT を作る王道手順

### ① 基準マップの準備

```matlab
id_vec = -400:20:  0;   % A
iq_vec =   0 :20: 600;  % A
[ID, IQ] = ndgrid(id_vec, iq_vec);

phi_d_map = ... ; % [length(id_vec) x length(iq_vec)]
phi_q_map = ... ; %   〃
```

### ② 1 次導関数（J）を中央差分

```matlab
d_id = id_vec(2) - id_vec(1);
d_iq = iq_vec(2) - iq_vec(1);

Ldd = diff(phi_d_map,2,1)/(2*d_id);                  % ∂φd/∂id
Ldq = diff(phi_d_map,1,2)/d_iq;  Ldq = Ldq(:,2:end-1);% ∂φd/∂iq
Lqd = diff(phi_q_map,2,1)/(2*d_id);                  % ∂φq/∂id
Lqq = diff(phi_q_map,1,2)/d_iq;  Lqq = Lqq(:,2:end-1);
```

*(境界は 1 次差分 or 外挿で補完)*

### ③ 2 次導関数（Hd, Hq）

```matlab
% φd
Hd11 = diff(phi_d_map,2,1)/(d_id^2);                 % ∂²φd/∂id²
Hd22 = diff(phi_d_map,2,2)/(d_iq^2);                 % ∂²φd/∂iq²
Hd12 = diff(diff(phi_d_map,1,1),1,2)/(d_id*d_iq);    % mixed
Hd = cat(3, Hd11, Hd12, permute(Hd12,[2 1 3]), Hd22);% 2×2×N LUT

% φq
Hq11 = diff(phi_q_map,2,1)/(d_id^2);
Hq22 = diff(phi_q_map,2,2)/(d_iq^2);
Hq12 = diff(diff(phi_q_map,1,1),1,2)/(d_id*d_iq);
Hq = cat(3, Hq11, Hq12, permute(Hq12,[2 1 3]), Hq22);
```

* `cat(3,…)` で **2×2×N** の 3-D 配列にしておくと look-up が楽
* 仕上げに **`griddedInterpolant`** を作る

  ```matlab
  F_Hd11 = griddedInterpolant(ID(2:end-1,2:end-1), IQ(2:end-1,2:end-1), Hd11,'linear','nearest');
  …      % Hd12, Hq11, …
  ```

---

## 2️⃣ **スプライン / 多項式フィット**で微分する（滑らかさ重視）

```matlab
F_d  = csapi({id_vec, iq_vec}, phi_d_map);    % 2-D cubic spline
F_q  = csapi({id_vec, iq_vec}, phi_q_map);

% 任意点 (id0, iq0) での 2 次導関数
Hd11 = fnder(F_d, [2 0]);     Hd12 = fnder(F_d, [1 1]);
Hd22 = fnder(F_d, [0 2]);
Hq11 = fnder(F_q, [2 0]);     ...  % 同様
```

* `fnder` は Curve Fitting Toolbox
* スプラインは微分が解析的なので **境界を含め滑らか**
* 重い場合はオフラインでメッシュ上に **サンプリング→LUT 化** すれば良い

---

## 3️⃣ **物理モデルがある場合は解析式で直接**

例：

$$
 \phi_d = \psi_f + \bigl(L_{d0}+k_s\,i_d+k_c\,i_q^2\bigr)\,i_d  
$$

なら

$$
 Hd =
 \begin{bmatrix}
   2k_s & 2k_c\,i_q\\
   2k_c\,i_q & 2k_c\,i_d
 \end{bmatrix},
\quad
 H_q =
 \begin{bmatrix}
   0 & 2k_c\,i_d\\
   2k_c\,i_d & 2k_c\,i_q
 \end{bmatrix}
$$

* 係数 `k_s, k_c` は FEM から同定
* **計算負荷はごく軽い**のでリアルタイム再計算も可

---

## 4️⃣ Simulink での実用ワークフロー

```text
Prelookup (id,iq index)  ─┬─► 2-D Interp  φd    ─┐
                          ├─► 2-D Interp  φq    ┤(不要なら省略)
                          ├─► 2-D Interp  Ldd   │
                          ├─► 2-D Interp  …     │
                          └─► 2-D Interp  Hd11  ─┬─► MATLAB-Fcn (EKF)
                              2-D Interp  Hd12  ┤
                              2-D Interp  Hq11  ┘
```

* **Hd, Hq の 4 要素**を **Bus** でまとめると配線がスッキリ
* 初期開発は **MATLAB Function ブロック**内で差分を直接計算 → 後で LUT に置換

---

## まとめ — Hd, Hq の作り方は 3 通り

| 方法            | 特徴                   | 向き／不向き                      |
| ------------- | -------------------- | --------------------------- |
| ① 中央差分        | 実装が簡単。マップが粗いとノイズ大    | FEM メッシュが高密度 or 滑らかに後処理する場合 |
| ② スプライン / 多項式 | 滑らかで境界も綺麗。Toolbox 必須 | 実機測定点が疎でも OK／オンライン微分は重い     |
| ③ 解析式         | 計算が最軽量。パラメータ推定が別途必要  | モータモデルが明確に分かっている場合          |

いずれも **オフラインで 2×2×N の LUT を作り、EKF にはその時点の (id, iq) で補間した Hd, Hq を渡す**のが定石です。
