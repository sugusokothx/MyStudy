https://qiita.com/MoriKen/items/0c80ef75749977767b43

### 1. システムモデルの整理

IPMSM (d–q 軸) の電圧方程式を **回転子電気角速度 ω<sub>e</sub> \[rad/s]** を既知と仮定して書くと

$$
\begin{aligned}
v_d &= R_s\,i_d - \, \omega_e\,L_q\,i_q + L_d\frac{\mathrm d i_d}{\mathrm dt},\\
v_q &= R_s\,i_q + \, \omega_e\,L_d\,i_d + \omega_e\,\psi_f + L_q\frac{\mathrm d i_q}{\mathrm dt}.
\end{aligned}
$$

ここで

* $R_s$ : 固定子抵抗
* $\psi_f$ : 永久磁石鎖交磁束
* $L_d,L_q$ : d/q 軸インダクタンス

を「**未知**または徐々に変動する」パラメータとして扱いたいので，**状態ベクトル**を

$$
\mathbf x=\begin{bmatrix} i_d & i_q & R_s & \psi_f\end{bmatrix}^{\!\mathsf T}
$$

と定義します。

---

### 2. 連続時間状態方程式 → 離散化

電流微分を解いて

$$
\begin{aligned}
\dot i_d &=\frac1{L_d}\!\Bigl(-R_s i_d + \omega_e L_q i_q+v_d\Bigr),\\
\dot i_q &=\frac1{L_q}\!\Bigl(-R_s i_q - \omega_e L_d i_d - \omega_e\psi_f + v_q\Bigr),\\
\dot R_s &= 0,\qquad
\dot\psi_f = 0
\end{aligned}
$$

と書けます（$R_s,\,\psi_f$ は**ランダムウォーク**でモデル化）。
離散時間 $k\to k+1$ に **前進オイラー**で写像すると

$$
\mathbf x_{k+1}=f(\mathbf x_k,\mathbf u_k)\;=\;
\mathbf x_k+\dot{\mathbf x_k}\,T_s
$$

$T_s$ はサンプリング周期（コード中の `dt`）。

---

### 3. 観測モデル

観測できるのは電流センサによる

$$
\mathbf z_k=\begin{bmatrix}i_{d,\text{obs}}\\ i_{q,\text{obs}}\end{bmatrix}
=H\mathbf x_k+ \mathbf v_k,\qquad
H=\begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix},
$$

で，測定雑音 $\mathbf v_k\sim\mathcal N(\mathbf 0,R)$。

---

### 4. 拡張カルマンフィルタ (EKF) の設計フローとコードの対応

| EKF ステップ                    | 数式                   | コード片                                 |                                                                                    |                         |                                             |                   |                          |                                                                                                            |                                                            |
| --------------------------- | -------------------- | ------------------------------------ | ---------------------------------------------------------------------------------- | ----------------------- | ------------------------------------------- | ----------------- | ------------------------ | ---------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| **予測 (time-update)**        | (\hat{\mathbf x}\_{k | k-1}=f(\hat{\mathbf x}\_{k-1         | k-1},\mathbf u\_{k-1}))<br>(F\_k=\left.\dfrac{\partial f}{\partial\mathbf x}\right | *{\hat{\mathbf x}*{k-1  | k-1}})<br>(P\_{k                            | k-1}=F\_k P\_{k-1 | k-1} F\_k^{\mathsf T}+Q) | `predict()` 内：<br> \* `x_dot` が $f$<br> \* `F` がヤコビアン，`F_k = I+F⋅dt`<br> \* `self.P = F_k @ P @ F_k.T + Q` |                                                            |
| **更新 (measurement-update)** | (S\_k=H P\_{k        | k-1} H^{\mathsf T}+R)<br>(K\_k=P\_{k | k-1} H^{\mathsf T} S\_k^{-1})<br>(\hat{\mathbf x}\_{k                              | k}= \hat{\mathbf x}\_{k | k-1}+K\_k(\mathbf z\_k-H\hat{\mathbf x}\_{k | k-1}))<br>(P\_{k  | k}=(I-K\_k H)P\_{k       | k-1})                                                                                                      | `update()` 内：<br> \* `S`, `K`, `y`, `self.x_hat`, `self.P` |

#### ヤコビアン $F$ の内訳（コード中 `F`）

$$
F=
\begin{bmatrix}
-\dfrac{R_s}{L_d} & \dfrac{\omega_e L_q}{L_d} & -\dfrac{i_d}{L_d} & 0\\[6pt]
-\dfrac{\omega_e L_d}{L_q} & -\dfrac{R_s}{L_q} & -\dfrac{i_q}{L_q} & -\dfrac{\omega_e}{L_q}\\[6pt]
0&0&0&0\\
0&0&0&0
\end{bmatrix}.
$$

上 2 行は電流方程式を偏微分したもので，下 2 行は $R_s,\,\psi_f$ のダイナミクスがゼロ（＝ランダムウォーク）であることを表します。

---

### 5. ノイズ共分散 $Q,\,R$ の選び方

* **$Q$** – モデル化誤差やパラメータゆっくり変動を許容する「柔らかさ」。
  本コードでは電流よりも $R_s,\,\psi_f$ の行に非常に小さな値を置き，
  *「急激には変わらないが，温度変化で数十秒スケールなら動く」* という prior を反映。
* **$R$** – 電流センサのノイズ分散。データシートや実測で決める。

$Q$ を大きくすると推定がパラメータ変動にすばやく追随する一方，ノイズで揺れやすくなる。
$R$ を小さくするとセンサを信じやすくなるが，過小評価すると発散しやすい。

---

### 6. 温度推定ロジック

推定した $\hat\psi_f$ を **実験で得た一次近似**

$$
T_{\text{rotor}} = a\,\psi_f + b
$$

に代入（`get_estimated_temp`）。フィルタ設計とは独立の**後段推定器**ですが，
$\psi_f$ を正確に捕まえる EKF あってこその推定です。

---

### 7. まとめ ― EKF で何が嬉しいか

* **非線形**（$i_d i_q$ 間の結合，$\omega_e \psi_f$ 項）でも**逐次線形化**で統一的に設計可。
* パラメータを状態拡張することで **計測できない物理量（抵抗・磁束）を同時推定**。
* 温度や磁束の **緩やかな時間変動** を $Q$ 調整だけで吸収できるため，実機への適用が容易。

> 同じ手順でインダクタンスや磁極位置 (θ) を追加した EKF / UKF も組めます。
> モデルの選び方（ランダムウォークか，より物理的な温度モデルか）は
> **「どのくらい速く変わる量か・観測で十分拘束されるか」** を基準に決めるのがコツです。

これがコード中の実装と EKF 理論の対応関係です。原理式を押さえておけば，
**状態数を増やす／別のモータへ適用する** ときも同じ設計フローで拡張できます。
