# IPMSM_Simulator ― クラス説明書  
*Interior Permanent-Magnet Synchronous Motor (IPMSM) ― 簡易熱依存パラメータ付きシミュレータ*

---

## 1. 目的  
本クラスは **IPMSM** の *dq* 軸モデルをオイラー前進法で離散化し、  
ロータ温度上昇による **巻線抵抗 $R_s$** および **永久磁石フラックス $\psi_f$** の変動を再現します。  
制御アルゴリズムやオブザーバの検証用に、真値電流 $i_d,i_q$ をオンライン生成します。

---

## 2. 物理モデル  

### 2.1 *dq* 軸電圧方程式（連続時間）  

$$
\begin{aligned}
\frac{d i_d}{dt} &= \frac{1}{L_d}\Bigl(-R_s\,i_d + \omega_e L_q i_q + v_d\Bigr),\\[4pt]
\frac{d i_q}{dt} &= \frac{1}{L_q}\Bigl(-R_s\,i_q - \omega_e L_d i_d - \omega_e \psi_f + v_q\Bigr),
\end{aligned}
$$

| 記号 | 意味 | 単位 |
|------|------|------|
| $i_d,i_q$ | *dq* 軸電流 | A |
| $v_d,v_q$ | *dq* 軸電圧 | V |
| $L_d,L_q$ | *dq* 軸インダクタンス | H |
| $R_s$ | 巻線抵抗（温度依存） | Ω |
| $\psi_f$ | 永久磁石フラックス（温度依存） | Wb |
| $\omega_e$ | 電気角速度 | rad/s |

### 2.2 温度モデル  

ロータ温度 $T_\text{rotor}$ は単純に  
$$T_\text{rotor}(t)=25+ t\;[^\circ\mathrm{C}] \qquad (t:\ \mathrm{s})$$  
と仮定し、$85^\circ\mathrm{C}$ で頭打ちさせます。

温度係数によるパラメータ補正:

$$
\begin{aligned}
R_s(T)   &= R_{s0}\,\bigl[1 + \alpha_R\,(T-25)\bigr],\\
\psi_f(T) &= \psi_{f0}\,\bigl[1 + \alpha_\psi\,(T-25)\bigr],
\end{aligned}
$$

* $\alpha_R = 0.00393\;\mathrm{^{\circ}C^{-1}}$ : 銅の抵抗温度係数  
* $\alpha_\psi = -0.0008\;\mathrm{^{\circ}C^{-1}}$ : 磁束の温度減少係数

---

## 3. 離散化 ― オイラー前進法  

時間刻み $\Delta t$ で

$$
\begin{aligned}
i_d[k+1] &= i_d[k] + \Delta t\,\left.\frac{d i_d}{dt}\right|_{k} \\[4pt]
i_q[k+1] &= i_q[k] + \Delta t\,\left.\frac{d i_q}{dt}\right|_{k}
\end{aligned}
$$

---

## 4. クラス構造と主メソッド  

| メソッド | 概要 |
|----------|------|
| `__init__(dt,Ld,Lq,pole_pairs)` | 定数パラメータと初期状態を設定 |
| `update_true_parameters(t)` | 温度に応じて $R_s,\psi_f$ を更新 |
| `step(v_d,v_q,we)` | 1 ステップ分の $i_d,i_q$ を計算して返す |

### 4.1 時系列シミュレーション例  

```python
sim = IPMSM_Simulator(dt=1e-4, Ld=1e-3, Lq=1.4e-3, pole_pairs=4)
for k in range(N):
    t = k * sim.dt
    sim.update_true_parameters(t)
    id_k, iq_k = sim.step(v_d_cmd, v_q_cmd, omega_e_cmd)



