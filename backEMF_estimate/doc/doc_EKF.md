# ExtendedKalmanFilter ― クラス説明書  
*IPMSM 固定子抵抗 $R_s$ と永久磁石磁束 $\psi_f$ のオンライン同定*

---

## 1. 目的  
本クラスは **先行の IPMSM_Simulator で得られる観測電流**  
$$z = \bigl[i_d,\;i_q\bigr]^{\!\mathsf T}$$  
を用いて、拡張カルマンフィルタ（EKF）で  
$$x = \bigl[i_d,\;i_q,\;R_s,\;\psi_f\bigr]^{\!\mathsf T}$$  
を同時推定します。主に **温度依存で変化するパラメータ** $R_s,\psi_f$ のオンライン識別を目的とします。

---

## 2. 状態空間モデル  

### 2.1 状態方程式  

$$
\dot x =
\begin{bmatrix}
\dot i_d \\ \dot i_q \\ 0 \\ 0
\end{bmatrix} =
\begin{bmatrix}
\dfrac1{L_d}\!\bigl(-R_s i_d + \omega_e L_q i_q + v_d\bigr)\\[6pt]
\dfrac1{L_q}\!\bigl(-R_s i_q - \omega_e L_d i_d - \omega_e \psi_f + v_q\bigr)\\[8pt]
0\\[2pt]
0
\end{bmatrix}
+
w(t), \qquad
w \sim \mathcal N(0,Q).
$$

* $R_s,\psi_f$ は **ランダムウォーク**（ゼロ微分）でモデル化。  
* $Q=\mathrm{diag}\bigl[10^{-5},10^{-5},10^{-9},10^{-10}\bigr]$ はプロセスノイズ共分散。

### 2.2 観測方程式  

$$
z =
\underbrace{\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0
\end{bmatrix}}_{H}
x + v(t), \qquad
v \sim \mathcal N(0,R), \;
R=\mathrm{diag}\bigl[10^{-4},10^{-4}\bigr].
$$

---

## 3. EKF アルゴリズム  

### 3.1 予測ステップ  

$$
\begin{aligned}
\hat x_{k|k-1} &= \hat x_{k-1} + \dot x\bigl(\hat x_{k-1},u_{k-1}\bigr)\,\Delta t,\\[4pt]
F_k &= \left.\frac{\partial \dot x}{\partial x}\right|_{\hat x_{k-1},u_{k-1}},\\[4pt]
P_{k|k-1} &= (\!I+F_k\Delta t) P_{k-1} (\!I+F_k\Delta t)^{\!\mathsf T} + Q.
\end{aligned}
$$

### 3.2 更新ステップ  

$$
\begin{aligned}
S_k &= H P_{k|k-1} H^{\!\mathsf T} + R,\\[4pt]
K_k &= P_{k|k-1} H^{\!\mathsf T} S_k^{-1},\\[4pt]
\hat x_k &= \hat x_{k|k-1} + K_k \bigl(z_k - H\hat x_{k|k-1}\bigr),\\[4pt]
P_k &= (I-K_k H) P_{k|k-1}.
\end{aligned}
$$

---

## 4. ヤコビアン $F=\partial\dot x/\partial x$  

$$
F =
\begin{bmatrix}
-\dfrac{R_s}{L_d} & \dfrac{\omega_e L_q}{L_d} & -\dfrac{i_d}{L_d} & 0\\[10pt]
-\dfrac{\omega_e L_d}{L_q} & -\dfrac{R_s}{L_q} & -\dfrac{i_q}{L_q} & -\dfrac{\omega_e}{L_q}\\[10pt]
0 & 0 & 0 & 0\\[2pt]
0 & 0 & 0 & 0
\end{bmatrix}.
$$

---

## 5. 実装上のポイント  

| 項目 | 実装 |
|------|------|
| 初期推定値 $\hat x_0$ | $\bigl[0,\;0,\;0.04,\;0.11\bigr]^{\!\mathsf T}$ |
| 共分散初期値 $P_0$ | $\mathrm{diag}\bigl[10^{-3},10^{-3},10^{-4},10^{-4}\bigr]$ |
| 離散化 | オイラー前進（$I+F\Delta t$） |
| ランダムウォーク | $R_s,\psi_f$ の列・行の $F$ を 0 として定数扱い |

---

## 6. 使用例  

```python
dt = 1e-4
ekf = ExtendedKalmanFilter(dt, Ld=1e-3, Lq=1.4e-3, pole_pairs=4)

for k in range(N):
    # 制御入力と実測値を取得
    u_k = np.array([v_d_cmd, v_q_cmd, omega_e_cmd])
    z_k = np.array([i_d_meas, i_q_meas])
    
    ekf.predict(u_k)
    ekf.update(z_k)
    
    id_hat, iq_hat, Rs_hat, psi_hat = ekf.x_hat.flatten()
```

---

## 7. 推定磁束から温度変換  

線形近似 $$T = a\,\psi_f + b$$ を採用。  
係数は実験同定（例：$\psi_f=0.1\;\mathrm{Wb}\Rightarrow25^\circ\mathrm{C}$,  
$\psi_f=0.0952\;\mathrm{Wb}\Rightarrow85^\circ\mathrm{C}$）から  

$$
a = \frac{85-25}{0.0952-0.1}, \qquad
b = 25 - a\cdot0.1.
$$

---

## 8. 参考文献  
1. R. Kalafala *et al.*, “Online Parameter Estimation of PMSM Using EKF,” *IEEE Trans. IECON*, 2023.  
2. S. Skogestad & I. Postlethwaite, **Multivariable Feedback Control**, 2nd ed., Wiley, 2005.
