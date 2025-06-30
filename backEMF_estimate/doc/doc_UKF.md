# UnscentedKalmanFilter ― クラス説明書  
*IPMSM 固定子抵抗 $R_s$ と永久磁石磁束 $\psi_f$ のオンライン同定 (UKF 版)*

---
## 1. 目的
$z = \bigl[i_d,\;i_q\bigr]^{\!\mathsf T}$ を観測量とし，
$x = \bigl[i_d,\;i_q,\;R_s,\;\psi_f\bigr]^{\!\mathsf T}$ を
Unscented Kalman Filter (UKF) で推定します。


<!-- ## 1. 目的  
$$z = \bigl[i_d,\;i_q\bigr]^{\!\mathsf T}$$ を観測量とし，  
$$x = \bigl[i_d,\;i_q,\;R_s,\;\psi_f\bigr]^{\!\mathsf T}$$  
を **Unscented Kalman Filter (UKF)** で推定します。   -->


EKF と比べ，ヤコビアン計算を避けつつ高次精度で非線形性を扱えるのが利点です。

---

## 2. UKF パラメータ  

| 記号 | 意味 | 本実装値 |
|------|------|---------|
| $\alpha$ | シグマ点の拡張係数 | $10^{-3}$ |
| $\beta$ | 事前知識 ($\beta=2$: 正規分布最適) | $2$ |
| $\kappa$ | 二次モーメント調整 | $0$ |
| $\lambda = \alpha^2(n+\kappa)-n$ | スケーリング | 近似 $-n$ |

$$
w_0^{(m)} = \frac{\lambda}{n+\lambda},\quad
w_0^{(c)} = w_0^{(m)} + 1-\alpha^2+\beta,\quad
w_i^{(m)} = w_i^{(c)} = \frac{1}{2(n+\lambda)}\;(i\ge1).
$$

---

## 3. モデル

### 3.1 状態遷移 $f(x,u)$  

$$
\begin{aligned}
i_d^{+} &= i_d + \frac{\Delta t}{L_d}\Bigl(-R_s i_d + \omega_e L_q i_q + v_d\Bigr),\\[6pt]
i_q^{+} &= i_q + \frac{\Delta t}{L_q}\Bigl(-R_s i_q - \omega_e L_d i_d - \omega_e \psi_f + v_q\Bigr),\\[6pt]
R_s^{+} &= R_s,\qquad
\psi_f^{+} = \psi_f.
\end{aligned}
$$

ここで $^{+}$ は次ステップ値。$R_s,\psi_f$ は **ランダムウォーク** として保持。  

### 3.2 観測関数 $h(x)$  

$$
h(x) =
\begin{bmatrix}
i_d\\ i_q
\end{bmatrix}.
$$

---

## 4. UKF アルゴリズム  

### 4.1 シグマ点生成  

1. $$\chi_0 = \hat x_{k-1}$$  
2. コレスキー分解 $$\sqrt{(n+\lambda)P_{k-1}}$$ を列 $\mathbf a_i$ とすると  
   $$\chi_i = \hat x_{k-1} + \mathbf a_i,\qquad
     \chi_{i+n} = \hat x_{k-1} - \mathbf a_i.$$

### 4.2 予測ステップ  

$$
\begin{aligned}
\chi_i^- &= f(\chi_i,u_{k-1}),\\
\hat x_{k|k-1} &= \sum_{i=0}^{2n} w_i^{(m)} \chi_i^-,\\
P_{k|k-1} &= \sum_{i=0}^{2n} w_i^{(c)}
           \bigl(\chi_i^- - \hat x_{k|k-1}\bigr)\bigl(\chi_i^- - \hat x_{k|k-1}\bigr)^{\!\mathsf T}
           + Q.
\end{aligned}
$$

### 4.3 更新ステップ  

$$
\begin{aligned}
Z_i &= h(\chi_i^-),\\
\hat z_k &= \sum_{i} w_i^{(m)} Z_i,\\
S &= \sum_i w_i^{(c)} (Z_i-\hat z_k)(Z_i-\hat z_k)^{\!\mathsf T}+R,\\
T &= \sum_i w_i^{(c)} (\chi_i^- - \hat x_{k|k-1})(Z_i-\hat z_k)^{\!\mathsf T},\\
K &= T S^{-1},\\
\hat x_{k} &= \hat x_{k|k-1} + K\bigl(z_k-\hat z_k\bigr),\\
P_{k} &= P_{k|k-1} - K S K^{\!\mathsf T}.
\end{aligned}
$$

---

## 5. 実装ハイライト  

| 項目 | 設定値／方法 |
|------|--------------|
| 初期 $\hat x_0$ | $\bigl[0,\;0,\;0.04,\;0.11\bigr]^{\!\mathsf T}$ |
| $P_0$ | $\mathrm{diag}[10^{-3},10^{-3},10^{-4},10^{-4}]$ |
| $Q$ | $\mathrm{diag}[10^{-5},10^{-5},10^{-9},10^{-10}]$ |
| $R$ | $\mathrm{diag}[10^{-4},10^{-4}]$ |
| コレスキー失敗時 | $P \leftarrow P+10^{-6}I$ で再試行 |

---

## 6. 使用例  

```python
dt = 1e-4
ukf = UnscentedKalmanFilter(dt, Ld=1e-3, Lq=1.4e-3,
                            pole_pairs=4, n_states=4)

for k in range(N):
    u_k = np.array([v_d_cmd, v_q_cmd, omega_e_cmd])
    z_k = np.array([i_d_meas, i_q_meas])
    
    ukf.predict(u_k)
    ukf.update(z_k)
    
    id_hat, iq_hat, Rs_hat, psi_hat = ukf.x_hat.flatten()
```

---

## 7. 推定磁束から温度変換  

線形近似 $$T = a\,\psi_f + b$$ を採用（実験同定）。  

$$
a = \frac{85-25}{0.0952-0.1}, \qquad
b = 25 - a\cdot0.1.
$$

---

## 8. 参考文献  
1. S. J. Julier & J. K. Uhlmann, “Unscented Filtering and Nonlinear Estimation,” *Proc. IEEE*, vol. 92, no. 3, pp. 401-422, 2004.  
2. R. Kalafala *et al.*, “Online Parameter Estimation of PMSM Using UKF,” *IEEE Trans. IECON*, 2024.  
