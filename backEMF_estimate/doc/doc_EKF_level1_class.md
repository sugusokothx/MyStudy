# Δφ-EKF ― クラス説明書

*PMSM 磁束マップ + Δφ 拡張カルマンフィルタ*

---

## 1. 目的

実機で計測済みの **磁束マップ**

$$
\bigl(\,i_d,i_q\bigr)\;\longmapsto\;\bigl(\phi_d^{\mathrm{map}},\;\phi_q^{\mathrm{map}}\bigr)
$$

を基準に、運転中に生じる **モデリング誤差 Δφ** と **固定子抵抗 $R_s$** を
オンライン推定します。

推定状態は

$$
x=\begin{bmatrix}
i_d \\[2pt] i_q \\[2pt] \Delta\phi_d \\[2pt] \Delta\phi_q \\[2pt] R_s
\end{bmatrix}\!,
\qquad
z=\begin{bmatrix}
i_d^{\mathrm{obs}} \\[2pt] i_q^{\mathrm{obs}}
\end{bmatrix}
$$

であり、**外部センサ**は電流 2 軸のみ。
エンコーダ不要の **トルク推定＋補償** を最終目的とします。

---

## 2. 状態空間モデル

### 2.1 磁束モデル

$$
\phi_d = \phi_d^{\mathrm{map}}\!\bigl(i_d,i_q\bigr) + \Delta\phi_d,  
\phi_q = \phi_q^{\mathrm{map}}\!\bigl(i_d,i_q\bigr) + \Delta\phi_q.
$$

$\phi_{d,q}^{\mathrm{map}}$ は実測 LUT を 2-次元補間
(**`RegularGridInterpolator`**) で評価。

### 2.2 連続時間状態方程式

$$
\dot x = f(x,u) + w,\qquad
u=\begin{bmatrix}v_d \\ v_q \\ \omega_e\end{bmatrix},
$$

$$
f(x,u)=
\begin{bmatrix}
\dfrac1{L_d}\bigl(-R_s i_d + \omega_e L_q i_q + v_d\bigr) \\[6pt]
\dfrac1{L_q}\bigl(-R_s i_q - \omega_e L_d i_d
                - \omega_e \psi_f^{\mathrm{nom}} + v_q\bigr) \\[6pt]
0 \\[2pt] 0 \\[2pt] 0
\end{bmatrix},
$$

* $\psi_f^{\mathrm{nom}}$: LUT の $(i_d,i_q)=(0,0)$ での
  $q$-軸磁束を名目永久磁石磁束とみなす。
* $\Delta\phi_{d,q},\,R_s$ は **ランダムウォーク**。

### 2.3 観測方程式

$$
z = Hx + v,\qquad
H=
\begin{bmatrix}
1&0&0&0&0\\0&1&0&0&0
\end{bmatrix}.
$$

---

## 3. EKF アルゴリズム

| ステップ | 式 | 実装 |
|---------|------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| 予測    | $\hat x_{k\mid k-1}= \hat x_{k-1}+f(\hat x_{k-1},u_{k-1})\Delta t$                                                                | オイラー前進 |
|         | $F_k=\left.\dfrac{\partial f}{\partial x}\right\rvert_{\hat x_{k-1},u_{k-1}}$                                                     | 前進差分 (`numerical_jacobian`) |
|         | $P_{k\mid k-1}=(I+F_k\Delta t)P_{k-1}(I+F_k\Delta t)^\top+Q$                                                                     |               |
| 更新    | $S_k=HP_{k\mid k-1}H^\top+R$                                                                                                      |               |
|         | $K_k=P_{k\mid k-1}H^\top S_k^{-1}$                                                                                                |               |
|         | $\hat x_{k}= \hat x_{k\mid k-1}+K_k\bigl(z_k-H\hat x_{k\mid k-1}\bigr)$                                                           |               |
|         | $P_k=(I-K_k H)P_{k\mid k-1}$                                                                                                      |               |


---

## 4. ヤコビアン $F=\partial f/\partial x$

解析的には次式（LUT 偏微分を無視した近似）ですが、
実装は **数値差分** で LUT 勾配を含め自動で評価します。

$$
F \approx
\begin{bmatrix}
-\dfrac{R_s}{L_d} & \dfrac{\omega_e L_q}{L_d} & -\dfrac1{L_d} & 0 & -\dfrac{i_d}{L_d}\\[10pt]
-\dfrac{\omega_e L_d}{L_q} & -\dfrac{R_s}{L_q} & 0 & -\dfrac1{L_q} & -\dfrac{i_q}{L_q}\\[10pt]
0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0
\end{bmatrix}.
$$

---

## 5. 実装上のポイント

| 項目           | 設定                                                                 |
| ------------ | ------------------------------------------------------------------ |
| プロセスノイズ $Q$  | `diag(1e-6, 1e-6, 1e-8, 1e-8, 1e-9)`                               |
| 観測ノイズ $R$    | `diag(1e-4, 1e-4)`                                                 |
| 共分散初期値 $P_0$ | `diag(1e-3, 1e-3, 1e-4, 1e-4, 1e-4)`                               |
| 離散化          | $I+F\Delta t$（一次精度）                                                |
| LUT 評価       | `RegularGridInterpolator(bounds_error=False)` → 範囲外も補間外挿           |
| 参考トルク式       | $\displaystyle T_e = \tfrac32 p \bigl(\phi_d i_q-\phi_q i_d\bigr)$ |

---

## 6. 使用例

```python
DT = 1e-4
Ld, Lq = 0.3e-3, 0.5e-3      # 実機値で置換
ekf = DeltaPhiEKF(DT, Ld, Lq)

v_d_cmd, v_q_cmd = 30.0, 80.0
omega_e = 2*np.pi*200        # 200 Hz

for k in range(20_000):
    ekf.predict((v_d_cmd, v_q_cmd, omega_e))

    # 実機ではセンサ値に置換
    z = np.array([ekf.i_d, ekf.i_q]) + \
        np.random.normal(0, np.sqrt(1e-4), 2)
    ekf.update(z)

    if k % 1000 == 0:
        print(f"{k:5d}: φd={ekf.phi_d:+.4f} Wb, "
              f"φq={ekf.phi_q:+.4f} Wb, "
              f"Te={ekf.torque:+.2f} N·m")
```

---

## 7. 拡張・応用アイデア

1. **磁束マップの外挿品質向上**

   * RBF 補間や 2D スプラインを併用して外挿時の滑らかさを確保。
2. **Δφ に一次減衰項を追加**

   * モデル誤差が時間とともに自然に戻る場合は
     $\dot{\Delta\phi}=-\lambda\Delta\phi$ を追加。
3. **$R_s$ 温度モデルとの融合**

   * 温度センサ併用で $R_s(T)$ を同定し、熱保護へ展開。
4. **UKF への置換**

   * LUT 非線形性が強い場合は無ヤコビアン化で数値安定性向上。

---

## 8. 参考文献

1. J. Holtz, “Sensorless Control of PMSM—Part II: Flux and Parameter Estimation,” *IEEE Trans. Ind. Electron.*, vol. 67, no. 7, pp. 5480-5491, 2020.
2. Y. Shang *et al.*, “Extended Kalman Filtering With Flux Maps for Real-Time Torque Estimation,” *IEMDC*, 2023.
