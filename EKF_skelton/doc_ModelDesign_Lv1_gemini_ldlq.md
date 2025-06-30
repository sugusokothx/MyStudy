## PMSM 磁束・d/q 軸インダクタンス同時推定のための拡張カルマンフィルタ（EKF）設計書

### 1. はじめに
本設計書は、永久磁石同期モータ（PMSM）の磁束 $\psi_f$ に加え、  
d 軸・q 軸インダクタンス $L_d,\,L_q$ をオンライン推定する拡張カルマンフィルタ（EKF）の  
理論設計とアルゴリズム定式化をまとめたものである。  
これにより、**突極性 PMSM** に対し高精度パラメータ推定とロバスト制御を実現する。

---

### 2. 理論的基礎：突極性 PMSM の拡張状態空間モデリング

#### 2.1 d–q 軸電圧モデル
突極性 PMSM の電気ダイナミクス（d–q 座標）は次式で記述される。

\[
\begin{aligned}
v_d &= R_s i_d + L_d \frac{di_d}{dt} - p\Omega\,L_q i_q \\
v_q &= R_s i_q + L_q \frac{di_q}{dt} + p\Omega\,L_d i_d + p\Omega\,\psi_f
\end{aligned}
\]

* $v_d,\,v_q$ : d–q 軸電圧  
* $i_d,\,i_q$ : d–q 軸電流  
* $R_s$ : 固定子抵抗  
* $L_d,\,L_q$ : d–q インダクタンス  
* $p$ : 極対数 $\Omega$ : 機械角速度  
* $\psi_f$ : 永久磁石磁束

#### 2.2 パラメータ推定用の状態拡張
推定対象を **電流 $(i_d,i_q)$ ＋ 磁束 $\psi_f$ ＋ インダクタンス $(L_d,L_q)$** とし，  
拡張状態ベクトルを

\[
x = \begin{bmatrix} i_d & i_q & \psi_f & L_d & L_q \end{bmatrix}^{\!\top}
\]

と定義する。  
$\psi_f,\,L_d,\,L_q$ は時間的に緩やかに変動すると仮定し，ランダムウォークでモデル化する。

\[
\begin{aligned}
\dot{\psi}_f &= w_{\psi_f} \\
\dot{L}_d   &= w_{L_d} \\
\dot{L}_q   &= w_{L_q}
\end{aligned}
\]

ここで $w_{\psi_f},w_{L_d},w_{L_q}$ は各パラメータに対応するプロセスノイズ。

#### 2.3 連続時間拡張状態方程式
入力ベクトル $u=\!\begin{bmatrix} v_d & v_q & \omega \end{bmatrix}^{\!\top}$  
（$\omega=p\Omega$ は電気角速度）とすると，

\[
\begin{cases}
\dot{i}_d = \dfrac{1}{L_d}\bigl(-R_s i_d + \omega L_q i_q + v_d\bigr) \\
\dot{i}_q = \dfrac{1}{L_q}\bigl(-R_s i_q - \omega L_d i_d - \omega\psi_f + v_q\bigr) \\
\dot{\psi}_f = 0 \\
\dot{L}_d = 0 \\
\dot{L}_q = 0
\end{cases}
\]

---

### 3. EKF アルゴリズム定式化

#### 3.1 離散化（サンプリング周期 $T_s$）
一次オイラー離散化により

\[
x_{k+1} = x_k + T_s\,f_c\bigl(x_k,u_k\bigr)
\]

具体的には

\[
\begin{aligned}
i_{d,k+1} &= i_{d,k} + \frac{T_s}{L_{d,k}}\bigl(-R_s i_{d,k} + \omega_k L_{q,k} i_{q,k} + v_{d,k}\bigr) \\
i_{q,k+1} &= i_{q,k} + \frac{T_s}{L_{q,k}}\bigl(-R_s i_{q,k} - \omega_k L_{d,k} i_{d,k} - \omega_k \psi_{f,k} + v_{q,k}\bigr) \\
\psi_{f,k+1} &= \psi_{f,k} \\
L_{d,k+1} &= L_{d,k} \\
L_{q,k+1} &= L_{q,k}
\end{aligned}
\]

#### 3.2 ヤコビアン行列
状態ヤコビアン $F_k = \partial f /\partial x\big|_{x=\hat{x}_{k|k}}$ は  
5 × 5 行列となる（以下に $x_4=L_d,\;x_5=L_q$ とおく）。



$F\_k = \begin{bmatrix}
   \frac{\partial f\_1}{\partial i\_d} & \frac{\partial f\_1}{\partial i\_q} & \frac{\partial f\_1}{\partial \psi\_f} & \frac{\partial f\_1}{\partial L\_d} & \frac{\partial f\_1}{\partial L\_q} \\
   \frac{\partial f\_2}{\partial i\_d} & \frac{\partial f\_2}{\partial i\_q} & \frac{\partial f\_2}{\partial \psi\_f} & \frac{\partial f\_2}{\partial L\_d} & \frac{\partial f\_2}{\partial L\_q} \\
   \frac{\partial f\_3}{\partial i\_d} & \frac{\partial f\_3}{\partial i\_q} & \frac{\partial f\_3}{\partial \psi\_f} & \frac{\partial f\_3}{\partial L\_d} & \frac{\partial f\_3}{\partial L\_q} \\
   \frac{\partial f\_4}{\partial i\_d} & \frac{\partial f\_4}{\partial i\_q} & \frac{\partial f\_4}{\partial \psi\_f} & \frac{\partial f\_4}{\partial L\_d} & \frac{\partial f\_4}{\partial L\_q} \\
   \frac{\partial f\_5}{\partial i\_d} & \frac{\partial f\_5}{\partial i\_q} & \frac{\partial f\_5}{\partial \psi\_f} & \frac{\partial f\_5}{\partial L\_d} & \frac{\partial f\_5}{\partial L\_q}
\end{bmatrix}$

---

> **メモ**  
> * $Q$（プロセスノイズ共分散）と $R$（観測ノイズ共分散）のチューニングが推定性能に大きく影響する。  
> * ヤコビアンは数値微分でも構わないが，計算速度が要求される場合は上記の解析式を実装した方が高速。  
> * $L_d,L_q$ が物理的にあり得ない値（負や極端に小さい値）へ収束しないよう，EKF 外側でバウンディング処理を入れると安定しやすい。
