## Δφ 方式に合わせた EKF 再設計 ― “まず **状態** と **予測式** を定める”

---

### 1. 選ぶ状態ベクトル  $x_k$

| idx | 物理量              | 意味                                                 | 役割                       |
| --- | ---------------- | -------------------------------------------------- | ------------------------ |
| 1   | $i_d$            | d 軸電流（推定値）                                         | 電圧→電流ダイナミクスを扱い、観測と直接つながる |
| 2   | $i_q$            | q 軸電流（推定値）                                         | 同上                       |
| 3   | $\Delta\!\phi_d$ | **基準磁束マップ** $\phi_d^{\mathrm{map}}(i_d,i_q)$ からの偏差 | 温度・個体差・経時変化を吸収           |
| 4   | $\Delta\!\phi_q$ | 同上（q 軸）                                            | 同上                       |

$$
x_k \;=\;
\begin{bmatrix}
i_d \\[4pt] i_q \\[4pt] \Delta\phi_d \\[4pt] \Delta\phi_q
\end{bmatrix}
\qquad(\text{4 状態})
$$

*ポイント*

* **電流を状態に含める**ことで，観測方程式は単純な恒等写像 (後述) になり，
  φ→i の逆写像を **予測ステップだけで使う** 構成にできます。
* $\Delta\phi$ は「**ゆっくり／小さく変わる**」と仮定し，予測では **ランダムウォーク**とする
  （= 時定数をモデルに入れない → 実装が軽い & 帯域は Q 行列で調整）。

---

### 2. 観測方程式 $z_k = h(x_k)$

測定値は電流センサの $i_d^{\text{meas}}, i_q^{\text{meas}}$ のみ：

$$
z_k =
\begin{bmatrix}
i_d^{\text{meas}} \\[4pt] i_q^{\text{meas}}
\end{bmatrix}
,\qquad
h(x_k) =
\begin{bmatrix}
i_d \\[4pt] i_q
\end{bmatrix}
,\qquad
H =
\begin{bmatrix}
1 & 0 & 0 & 0 \\[4pt]
0 & 1 & 0 & 0
\end{bmatrix}.
$$

---

### 3. 予測モデル $x_{k+1}=f(x_k,u_k)$

$$
u_k =
\begin{bmatrix}
v_d \\[4pt] v_q \\[4pt] \omega_e
\end{bmatrix}.
$$

---

#### 3.1 補助関数 ― **基準磁束マップとそのヤコビ行列**

```matlab
[phi_d0, phi_q0,  ...
  Ldd, Ldq, Lqd, Lqq] = flux_map_with_grad(id, iq);
%  phi_d0, phi_q0 : 基準磁束 [Wb]
%  Ldd = ∂φd0/∂id [H]   (セルフ Ld)
%  Ldq = ∂φd0/∂iq [H]   (クロス)
%  Lqd = ∂φq0/∂id [H]
%  Lqq = ∂φq0/∂iq [H]
```

（**メモ**：基準マップはオフライン計測や FEM から生成。
勾配 $L_{**}$ は中央差分 or 事前計算 LUT で持たせる。）

---

#### 3.2 **連続時間**方程式

1. **実磁束（推定値）**

   $$
   \phi_d = \phi_d^{\text{map}}(i_d,i_q) + \Delta\phi_d,\qquad
   \phi_q = \phi_q^{\text{map}}(i_d,i_q) + \Delta\phi_q
   $$

2. **dq 電圧方程式**

   $$
   \begin{aligned}
   \dot{\phi}_d &= v_d - R_s\,i_d + \omega_e\,\phi_q \\
   \dot{\phi}_q &= v_q - R_s\,i_q - \omega_e\,\phi_d
   \end{aligned}
   \tag{1}
   $$

3. **φ と i の関係（ヤコビ線形化）**

   $$
   \dot{\phi}
     = J_\phi(i_d,i_q)\,\dot{i}
     \quad\Longrightarrow\quad
   \dot{i}
     = J_\phi^{-1}\,\dot{\phi}
   $$

   ここで
   $J_\phi =
   \begin{bmatrix}
     L_{dd} & L_{dq}\\
     L_{qd} & L_{qq}
   \end{bmatrix}$.

4. **Δφ のダイナミクス（ランダムウォーク）**

   $$
     \dot{\Delta\phi_d}=0,\qquad
     \dot{\Delta\phi_q}=0
   $$

---

#### 3.3 **離散化**（前進オイラー： $Ts=200\,\mu s$ 例）

```matlab
% --- 1) 実磁束 ---
phi_d =  phi_d0 + dphi_d;
phi_q =  phi_q0 + dphi_q;

% --- 2) φ̇ (式 1) ---
phi_dot_d = vd - Rs*id + omega * phi_q;
phi_dot_q = vq - Rs*iq - omega * phi_d;

% --- 3) Jφ & その逆行列 ---
J = [Ldd, Ldq;
     Lqd, Lqq];
di_dt = J \ [phi_dot_d ; phi_dot_q];  % 2×1 ベクトル

% --- 4) 状態更新 ---
id_next      = id      + Ts * di_dt(1);
iq_next      = iq      + Ts * di_dt(2);
dphi_d_next  = dphi_d;               % ランダムウォーク (Q でノイズを注入)
dphi_q_next  = dphi_q;

x_next = [id_next; iq_next; dphi_d_next; dphi_q_next];
```

---

### 4. ヤコビ行列 $F = ∂f/∂x$ の組み立て

* **id, iq** 行は  $\partial \dot{i}/\partial i$ に **Jφ とその導関数** が含まれるため手計算は煩雑。
  実装は **数値差分（5 点差分 or 嵩上げされた係数付き中心差分）** が現実的です。
* **Δφ 行** は恒等 ⇒ $F_{3,3}=F_{4,4}=1$ で，他ゼロ。

> **Tips**
> 500 Hz 程度の EKF 帯域では数値ヤコビの時間コストは無視できる（MCU 240 MHz クラスで <5 µs／step）。
> 精度が足りなければ Ldd 等ヤコビの勾配を LUT 化し、部分解析＋部分差分でも OK。

---

### 5. Q・R 行列の初期案

| 状態                          | 典型値                            | コメント              |
| --------------------------- | ------------------------------ | ----------------- |
| $i_d,i_q$                   | $Q_{ii} = (0.5\ \text{A})^2$   | 電流モデル不確かさ         |
| $\Delta\phi_d,\Delta\phi_q$ | $Q_{\phi\phi}=1\times10^{-10}$ | 5–10 Hz 帯域ならこのオーダ |

```matlab
Q = diag([2.5e-1, 2.5e-1, 1e-10, 1e-10]);  % 単位: [A^2, A^2, Wb^2, Wb^2]
R = diag([ (0.2)^2 , (0.2)^2 ]);           % 12-bit ±400 A センサ例
```

---

## まとめ

1. **状態ベクトル**

   $$
     x = \bigl[i_d,\; i_q,\; \Delta\phi_d,\; \Delta\phi_q\bigr]^\top
   $$

2. **観測**：電流センサ → $H=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}$

3. **予測**

   * 基準磁束マップ $\phi^{\mathrm{map}}(i_d,i_q)$ とそのヤコビ $J_\phi$ を取得
   * 電圧方程式 (1) で $\dot{\phi}$ を計算
   * $\dot{i}=J_\phi^{-1}\dot{\phi}$ から $i_{k+1}$ を更新
   * $\Delta\phi$ はランダムウォーク

4. **F 行列**：Δφ 部は恒等、$i$ 部は数値ヤコビで OK

> これで **アプローチ (B)＝Δφ 方式** の骨格が決まりました。
> 次ステップでは、この数式をもとに `f_discrete_ipm` と `calculate_F_ipm` を書き換え、基準磁束 LUT／その勾配 LUT を用意すればスケルトンを直接改造できます。


