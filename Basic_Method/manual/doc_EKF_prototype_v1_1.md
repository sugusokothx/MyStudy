# EKFによるセンサレス制御への移行と実装概要

はい、既存のPLLベースのセンサレス制御を**拡張カルマンフィルター（EKF）**を使用した制御に改良することが可能です。EKFは非線形システムの状態推定に非常に強力なツールであり、**モーター制御のようなアプリケーションに適しています**。

---

## 実装概要

以下に、提供されたコードをEKFベースのセンサレス制御に変換したPythonコードの内容をまとめます。

### 変更と追加点

#### EKFクラスの導入

- `EKF` クラスを定義し、状態推定のロジック（**予測ステップ**と**更新ステップ**）をカプセル化。

#### 状態ベクトルの定義

推定対象の状態は以下のとおりです：

$$
\hat{x} = \begin{bmatrix}
\theta \\
\omega \\
i_d \\
i_q
\end{bmatrix}
$$

- $\theta$: 電気角  
- $\omega$: 電気角速度  
- $i_d$, $i_q$: d-q軸電流

#### システムモデル

- モーターの電圧方程式と角速度のダイナミクスに基づいて構築。
- 本実装では、$\frac{d\omega}{dt} = 0$（角速度は一定）と仮定。

#### 測定モデル

- 測定値として $\alpha$-$\beta$ 軸の電流（$i_\alpha, i_\beta$）を使用。
- dq → αβ変換を用いて測定モデルを構築。

#### ヤコビアン行列の計算

- 予測ステップと更新ステップで使用するヤコビアン行列 $F$（システム）と $H$（測定）を明示的に導出・計算。

---

### EKFパラメータの初期化

| パラメータ名 | 内容 |
|--------------|------|
| `initial_x_ekf` | 初期状態推定値 |
| `initial_P_ekf` | 初期共分散行列（大きめに設定し、不確かさを表現） |
| `Q_ekf` | プロセスノイズ共分散行列（モデルの誤差） |
| `R_ekf` | 測定ノイズ共分散行列（センサのノイズ） |

> ※ これらのノイズパラメータはEKF性能に大きく影響します。現実のシステムに合わせて調整が必要です。

---

### シミュレーションループの変更

- PLLとI/Q復調のロジックを、EKFの `predict` および `update` メソッドの呼び出しに置き換え。
- 印加電圧 $V_\alpha$, $V_\beta$ は、推定角度 $\hat{\theta}$ に基づいて生成。
- モーターモデルから得られる電流 $(i_\alpha^{\text{meas}}, i_\beta^{\text{meas}})$ をEKFに渡す。

---

### 可視化（プロット）の追加

- 推定された電気角速度 $\hat{\omega}$ と真の電気角速度 $\omega_{\text{true}}$ の比較をプロット。
- 高速回転時のロバストな推定性能を確認可能。

---

## 特徴と利点

このEKF実装では、**角度と速度の推定が同時に行われます**。

- PLLベースのシステムと比較して、EKFはよりロバストな推定を提供。
- 特に**ノイズが多い環境や、動的な条件下**での性能向上が期待できます。

---

## 次のステップ

### 1. パラメータチューニング

- `Q_ekf`, `R_ekf` を調整して、推定誤差が最小となるようにチューニング。

### 2. 速度ダイナミクスの改善

- 現在は角速度 $\omega$ を一定と仮定。
- より高精度を目指す場合は、**機械的ダイナミクス**（慣性、摩擦）をモデルに追加することを検討。

### 3. UKFの検討

- EKFで精度不足が見られた場合は、**Unscented Kalman Filter（UKF）**の導入も可能。
- UKFはヤコビアン不要で高精度だが、計算コストは高め。

### 4. 実機への適用

- シミュレーションでの良好な結果を踏まえ、実機への適用を検討。
- 実装の移行に向けて、ハードウェアリソースやリアルタイム性も考慮。

---

## 参考

- 実装例や数式展開をご希望の場合、Pythonコードや詳細な解説も提供可能です。
