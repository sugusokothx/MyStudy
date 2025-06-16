## Vector-Rotator-Assisted Filter（VRAF）の原理式

### 1.  複素表現  
$$
u_{\alpha}(t),\; u_{\beta}(t)
\;\;\Longrightarrow\;\;
u_p(t) \;=\; u_{\alpha}(t) \;+\; j\,u_{\beta}(t)
$$

---

### 2.  デモジュレーション（回転座標への写像）  
$$
\tilde u(t)
\;=\;
u_p(t)\,
e^{-\sigma\,j\omega t},
\qquad
\sigma =
\begin{cases}
+1 & (\text{正相成分を抽出する場合})\\[6pt]
-1 & (\text{逆相成分を抽出する場合})
\end{cases}
$$

---

### 3.  低域通過フィルタ  
$$
\tilde x(t)
\;=\;
\operatorname{LPF}\!\bigl\{\tilde u(t)\bigr\}
$$

---

### 4.  位相遅れ補償付きリモジュレーション（元の座標系へ復帰）  
$$
y_p(t)
\;=\;
\tilde x(t)\,
e^{+\sigma\,j\omega\,(t-\tau_d)}
$$

* \(\tau_d\)：LPF の群遅延（位相補償を行わない場合は \(\tau_d = 0\) とおく）

---

### 5.  実部・虚部の取り出し  
$$
\boxed{
\;y_{\alpha}(t)=\Re\{y_p(t)\},
\quad
y_{\beta}(t)=\Im\{y_p(t)\}\;}
$$

---

## （参考）離散時間表現  
$$
n = 0,1,2,\ldots,\qquad T_s = \frac{1}{F_s}
$$

$$
\begin{aligned}
u_p[n]      &= u_{\alpha}[n] + j\,u_{\beta}[n] \\[6pt]
\tilde u[n] &= u_p[n]\;e^{-\sigma\,j\omega nT_s} \\[6pt]
\tilde x[n] &= \text{IIR‐LPF}\bigl(\tilde u[n]\bigr) \\[6pt]
y_p[n]      &= \tilde x[n]\;e^{+\sigma\,j\omega(nT_s-\tau_d)}
\end{aligned}
$$

LPF は Butterworth\(b_k,a_k\) 係数で

$$
\tilde x[n] - \sum_{k=1}^{M} a_k\,\tilde x[n-k]
\;=\;
\sum_{k=0}^{N} b_k\,\tilde u[n-k]
$$

を満たす。
