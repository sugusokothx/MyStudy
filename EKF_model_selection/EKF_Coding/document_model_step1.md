# EKF Δφ (Step‑1) モデル説明資料

## 1 . 目的

拡張カルマンフィルタ（EKF）を用いた Δφ（磁束偏差）一次モデル *ekf\_delta\_phi\_estimator.m* の動作原理と実装仕様をまとめる。本資料では **状態の定義**、**入力**、**観測値**、および **推定モデル**（状態・測定方程式とそのヤコビ行列）を中心に説明する。

---

## 2 . 状態ベクトル $\mathbf{x}$

| 記号               | 物理量     | 単位 | 説明                          |
| ---------------- | ------- | -- | --------------------------- |
| $i_d$            | d 軸電流   | A  | センサ測定値 `id_meas` のフィルタ済み推定値 |
| $i_q$            | q 軸電流   | A  | センサ測定値 `iq_meas` のフィルタ済み推定値 |
| $\Delta\!\phi_d$ | d 軸磁束偏差 | Wb | ベースマップ値からのずれ                |
| $\Delta\!\phi_q$ | q 軸磁束偏差 | Wb | ベースマップ値からのずれ                |

ベクトル表記：

$$
\mathbf{x} = \begin{bmatrix} i_d & i_q & \Delta\!\phi_d & \Delta\!\phi_q \end{bmatrix}^T \in \mathbb{R}^4
$$

初期推定値は $[0\ 0\ 0\ 0]^T$ 。電流推定に対する不確かさは小さく、磁束偏差に対して大きめの分散を設定している（`P_est = diag([0.01, 0.01, 1e-2, 1e-2])`）。

---

## 3 . 入力ベクトル $\mathbf{u}$

| 記号       | 物理量     | 単位    | 説明                   |
| -------- | ------- | ----- | -------------------- |
| $v_d$    | d 軸電圧指令 | V     | インバータ出力電圧（デッドタイム補正後） |
| $v_q$    | q 軸電圧指令 | V     | 〃                    |
| $\omega$ | 電気角速度   | rad/s | `omega`（ロータ電気速度）     |

ベクトル表記：

$$
\mathbf{u} = \begin{bmatrix} v_d & v_q & \omega \end{bmatrix}^T \in \mathbb{R}^3
$$

---

## 4 . 観測ベクトル $\mathbf{z}$

| 記号                  | 物理量      | 単位 | 説明                |
| ------------------- | -------- | -- | ----------------- |
| $i_d^{\text{meas}}$ | d 軸電流測定値 | A  | 電流センサ出力 `id_meas` |
| $i_q^{\text{meas}}$ | q 軸電流測定値 | A  | 電流センサ出力 `iq_meas` |

測定モデルは線形：

$$
\mathbf{z}_k = \mathbf{H}\,\mathbf{x}_k + \mathbf{v}_k, \quad
\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}
$$

ここで $\mathbf{v}_k \sim \mathcal{N}(0,\mathbf{R})$。既定の測定雑音共分散は `R = diag([1\!\times\!10^{-3}, 1\!\times\!10^{-3}])`。

---

## 5 . 推定モデル

### 5.1 状態方程式（離散）

電流微分方程式（パーク回路）に基づき、サンプル時間 $T_s$ で前進オイラー離散化する。

$$
\begin{aligned}
 i_d[k+1] &= i_d[k] + T_s\;\frac{1}{L_{d}^{\text{map}}}\Big( v_d[k] - R_s i_d[k] + \omega[k] \big( \phi_q^{\text{map}} + \Delta\!\phi_q[k] \big) \Big) \\[-1pt]
 i_q[k+1] &= i_q[k] + T_s\;\frac{1}{L_{q}^{\text{map}}}\Big( v_q[k] - R_s i_q[k] - \omega[k] \big( \phi_d^{\text{map}} + \Delta\!\phi_d[k] \big) \Big) \\[-1pt]
 \Delta\!\phi_d[k+1] &= \Delta\!\phi_d[k] \\[-1pt]
 \Delta\!\phi_q[k+1] &= \Delta\!\phi_q[k]
\end{aligned}
$$

* $\phi_{d/q}^{\text{map}}$\* とインダクタンス $L_{d/q}^{\text{map}}$ は外部 LUT から取得（Simulink から入力）。
* 磁束偏差はランダムウォークとみなし、動的モデルは恒等式。

式をまとめて

$$
\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k,\mathbf{u}_k) + \mathbf{w}_k,\quad \mathbf{w}_k\sim\mathcal{N}(0,\mathbf{Q})
$$

既定のプロセス雑音共分散：`Q = diag([1e-4, 1e-4, 1e-12, 1e-12])`。

### 5.2 ヤコビ行列 $\mathbf{F}=\tfrac{\partial \mathbf{f}}{\partial \mathbf{x}}$

連続時間ヤコビ行列 $\mathbf{A}$ を導き、一次近似で
$\mathbf{F} \approx \mathbf{I} + \mathbf{A}\,T_s$
とする。

$$
\mathbf{A} = \begin{bmatrix}
\tfrac{-R_s+\omega L_{qd}}{L_d} & \tfrac{\omega L_{qq}}{L_d} & 0 & \tfrac{\omega}{L_d} \\
\tfrac{-\omega L_{dd}}{L_q} & \tfrac{-R_s-\omega L_{dq}}{L_q} & \tfrac{-\omega}{L_q} & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \end{bmatrix}
$$

* $L_{dd}, L_{dq}, L_{qd}, L_{qq}$ は二階導関数（混合インダクタンス）で LUT から入力。

---

## 6 . EKF アルゴリズムフロー

1. **予測ステップ**
   $\hat{\mathbf{x}}_{k|k-1} = \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1},\mathbf{u}_{k-1})$
   $\mathbf{P}_{k|k-1} = \mathbf{F}\,\mathbf{P}_{k-1|k-1}\,\mathbf{F}^T + \mathbf{Q}$
2. **更新ステップ**
   イノベーション $\mathbf{y}_k = \mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}$
   カルマンゲイン $\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^T (\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R})^{-1}$
   $\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k\mathbf{y}_k$
   $\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}_k\mathbf{H})\mathbf{P}_{k|k-1}$

---

## 7 . パラメータ一覧（既定値）

| シンボル         | 値                               | 説明        |
| ------------ | ------------------------------- | --------- |
| $R_s$        | 37 mΩ                           | ステータ抵抗    |
| $T_s$        | 500 µs                          | サンプル周期    |
| $\mathbf{Q}$ | `diag([1e-4 1e-4 1e-12 1e-12])` | プロセス雑音共分散 |
| $\mathbf{R}$ | `diag([1e-3 1e-3])`             | 測定雑音共分散   |

---

## 8 . 実装ファイル構成

```
ekf_delta_phi_estimator.m      % EKF 本体
f_discrete_delta_phi.m         % 状態遷移関数 f
calculate_F_delta_phi.m        % ヤコビ行列 F の計算
```

---

## 9 . 今後の拡張アイデア

* **磁束偏差のダイナミクス導入** — Δφ をランダムウォークではなく一階遅れモデルで表現し、推定の収束速度を調整。
* **温度依存 Rs 推定の追加** — 抵抗変化を状態に含めることで高温時のモデル誤差を低減。
* **高次 EKF (Step‑2)** — 二次ヤコビやヘシアンを利用し線形化誤差をさらに抑制。

---

© 2025 Motor‑Control R\&D Team
