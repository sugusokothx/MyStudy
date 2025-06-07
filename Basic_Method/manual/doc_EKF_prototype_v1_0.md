# Transitioning from PLL-based Sensorless Control to an Observer-based Approach

あなたの現在のPLLベースのセンサレス制御は、停止〜低速域において堅実な性能を発揮しており、非常に良い出発点です。ここから、**Extended Kalman Filter（EKF）**や**Unscented Kalman Filter（UKF）**といったオブザーバを導入することで、よりダイナミックな条件下でも高精度な推定が可能になります。

---

## EKF vs. UKF for Sensorless Control

| 項目 | EKF | UKF |
|------|-----|-----|
| **原理** | 線形化したモデルを用いる（ヤコビアンが必要） | 非線形のまま「シグマ点」を用いて推定（ヤコビアン不要） |
| **メリット** | 計算量が少なく実績も豊富 | 高非線形系でも精度が高く、実装が比較的容易 |
| **デメリット** | 高非線形時に精度が劣る可能性 | 計算コストが高い |

---

## オブザーバベース制御への変換ステップ

どちらの手法を選ぶ場合でも、以下の手順で進めることができます。

### 1. 状態ベクトルの定義（$x$）

通常、以下のような状態を含めます：

$$
x = \begin{bmatrix}
\theta \\
\omega \\
i_d \\
i_q
\end{bmatrix}
$$

必要に応じて、磁束や抵抗などのパラメータを加えて拡張も可能です。

---

### 2. システムモデル $f(x, u)$（状態方程式）

PMSMの電気的ダイナミクスに基づきます：

$$
\frac{d}{dt}
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
=
\begin{bmatrix}
-\frac{R_s}{L_d} & -\omega \frac{L_q}{L_d} \\
\omega \frac{L_d}{L_q} & -\frac{R_s}{L_q}
\end{bmatrix}
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
+
\begin{bmatrix}
\frac{1}{L_d} & 0 \\
0 & \frac{1}{L_q}
\end{bmatrix}
\begin{bmatrix}
v_d \\
v_q
\end{bmatrix}
$$

$$
\frac{d}{dt} \theta = \omega
$$

$$
\frac{d}{dt} \omega = \text{（機械モデルを含めるか、ゆっくり変化と仮定）}
$$

---

### 3. 観測モデル $h(x)$（出力方程式）

$$
\begin{bmatrix}
i_\alpha \\
i_\beta
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}
i_d \\
i_q
\end{bmatrix}
$$

---

### 4. 雑音共分散行列の定義

- **プロセス雑音（$Q$）**：モデル誤差・ノイズに対応
- **観測雑音（$R$）**：センサノイズに対応

これらはチューニングが極めて重要です。

---

### 5. 予測・更新ステップの実装

- **予測ステップ**：状態と共分散を $f(x, u)$ により前進
- **更新ステップ**：観測 $z$ と $h(x)$ によって修正

---

## どちらを選ぶべきか？

まずは **EKFから始める** ことを推奨します：

- 実装とデバッグが比較的容易
- 多くのモータ制御の事例がEKFベース
- PMSMの非線形性はEKFで十分扱える場合が多い

精度や性能に不満があれば、次にUKFへと発展できます。

---

## 次のステップ

1. モータモデルの**離散化**：EKF/UKFでは通常、離散時間で計算
2. 雑音共分散 $Q$, $R$ の設計とチューニング
3. **PLLロジックをEKFまたはUKFに置き換え**
   - オブザーバ出力の $\hat{\theta}, \hat{\omega}$ をそのまま制御器に供給

---

シミュレーションでプロトタイプを検証することで、ハードウェア実装に移る前に多くの知見を得られます。

---

## 実装に進みますか？

EKFの数式展開・Python実装の詳細に進みたい場合は、そのままお知らせください。UKFの検討を続けることも可能です。
