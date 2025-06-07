# UKF ベースのセンサレス制御 ― Python プロトタイプ

以下は、**PLL 部分を Unscented Kalman Filter (UKF)** に置き換えた最小構成のシミュレータです。

---

## 状態ベクトル

状態変数 $x$ は以下のとおりです：

$$
x = \begin{bmatrix}
\theta_e \\
\omega_e \\
I_d \\
I_q
\end{bmatrix}
$$

- $\theta_e$：電気角  
- $\omega_e$：電気角速度  
- $I_d$：d軸電流  
- $I_q$：q軸電流

---

## 入力（αβ軸電圧）

$$
u = \begin{bmatrix}
V_\alpha \\
V_\beta
\end{bmatrix}
$$

- $V_\alpha$：α軸電圧  
- $V_\beta$：β軸電圧  

> ※ 入力は高周波（HF）注入電圧のみを想定しています。

---

## 状態方程式（Euler 離散化）

モーターのダイナミクスに基づき、状態は以下のように更新されます：

$$
I_d^+ = I_d + \frac{T_s}{L_d} \left( V_d - R_s I_d + \omega_e L_q I_q \right)
$$

$$
I_q^+ = I_q + \frac{T_s}{L_q} \left( V_q - R_s I_q - \omega_e L_d I_d \right)
$$

$$
\theta_e^+ = \mathrm{wrap}(\theta_e + \omega_e T_s)
$$

$$
\omega_e^+ = \omega_e \quad \text{（一定速度モデル）}
$$

> ※ dq電圧 $V_d$, $V_q$ は、$\alpha\beta$座標系の電圧 $V_\alpha$, $V_\beta$ を推定した電気角 $\hat{\theta}_e$ により回転変換して算出します。

---

## 観測方程式（αβ軸電流）

観測される電流は以下のように表されます：

$$
\begin{bmatrix}
I_\alpha \\
I_\beta
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & -\sin\theta_e \\
\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix}
I_d \\
I_q
\end{bmatrix}
$$

- $I_\alpha$：α軸電流  
- $I_\beta$：β軸電流  

> 観測値は、モーターの電気角に依存する実電流 $I_\alpha$, $I_\beta$（HF成分込み）です。

> 復調誤差信号やフィルター、PLLは UKF の内部ロジックに置き換えられ、**明示的には不要**となります。

---

## 実装上の補足

UKFはEKFと異なり、非線形性を扱うためにヤコビアン行列を**明示的に計算しません**。代わりに以下の方法を使用します：

### 1. シグマ点の生成

- 状態の平均と共分散を統計的に捉えるため、一連の**シグマ点**を生成します。

### 2. 非線形変換

- 生成されたシグマ点を、非線形な状態遷移関数（`_state_transition_function`）および観測関数（`_measurement_function`）に**直接通します**。

### 3. 重み付き平均と共分散

- 変換後のシグマ点から、**重み付きの平均と共分散**を計算し、新しい状態推定値と共分散行列を得ます。

> これにより、UKFは線形化誤差を回避し、**非線形性の強いシステムでも高精度な推定**が可能になります。

---

## UKFパラメータ

UKFでは以下のパラメータを使用してシグマ点の生成方法や重みを調整します：

- `alpha`：シグマ点の分布幅（小さい値ほど点が平均に近づく）
- `beta`：事前分布の仮定（通常は2）
- `kappa`：二次パラメータ（0や3-nが一般的）

> これらはUKFの挙動に大きく影響するため、**アプリケーションに応じたチューニング**が必要です。

---

## ノイズ共分散行列

- **プロセスノイズ共分散行列 $Q$**：モデルの不確かさを表現  
- **測定ノイズ共分散行列 $R$**：センサノイズ・観測誤差を反映

> $Q$, $R$ の設定は、UKFの性能にとって極めて重要です。

---

ご希望があれば、この構成を元にした Python 実装例も提供できます。
