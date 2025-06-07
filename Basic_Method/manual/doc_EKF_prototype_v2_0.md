# EKF ベースのセンサレス制御 ― Python プロトタイプ

以下は、**PLL 部分を Extended Kalman Filter (EKF)** に置き換えた**最小構成のシミュレータ**です。

---

## 状態ベクトル

状態変数 $x$ は以下のとおりです：

$$
x = \begin{bmatrix}
I_d \\
I_q \\
\theta_e \\
\omega_e
\end{bmatrix}
$$

---

## 入力（αβ軸電圧）

$$
u = \begin{bmatrix}
V_\alpha \\
V_\beta
\end{bmatrix}
$$

> ※ 入力は高周波（HF）注入電圧のみを想定。

---

## 状態方程式（Euler 離散化）

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

> ※ dq電圧 $V_d$, $V_q$ は、$\alpha\beta$座標系の電圧 $V_\alpha$, $V_\beta$ を $\theta_e$ で回転変換して算出。

---

## 観測方程式（αβ軸電流）

$$
\begin{bmatrix}
I_\alpha \\
I_\beta
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
\cos\theta_e & -\sin\theta_e \\
\sin\theta_e & \cos\theta_e
\end{bmatrix}
}_{R(\theta_e)}
\begin{bmatrix}
I_d \\
I_q
\end{bmatrix}
$$

- 観測値は実電流 $I_\alpha$, $I_\beta$（= HF成分込み）です。
- **復調誤差信号やフィルタ、PLLは不要**としています。

---

## 実装上の補足

- **Jacobian行列**は、解析式ではなく数値微
