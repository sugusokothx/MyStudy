### 離散 d–q モデル（IPMSM, sampling period \(T_s\)）

$$
\begin{aligned}
i_d[k+1] &= i_d[k] + \frac{T_s}{L_d}
           \Bigl(\,v_d[k] - R_s\,i_d[k] + \omega_e L_q\,i_q[k]\Bigr) \\
i_q[k+1] &= i_q[k] + \frac{T_s}{L_q}
           \Bigl(\,v_q[k] - R_s\,i_q[k] - \omega_e L_d\,i_d[k] - \omega_e\psi_f\Bigr)
\end{aligned}
$$

### Dead-beat 電圧指令（「1 サンプル後の目標電流」を実現）

$$
\begin{aligned}
v_d[k] &= R_s\,i_d[k] - \omega_e L_q\,i_q[k]
         + \frac{L_d}{T_s}\,\bigl(i_d^{\star}[k+1]-i_d[k]\bigr) \\[6pt]
v_q[k] &= R_s\,i_q[k] + \omega_e L_d\,i_d[k] + \omega_e\psi_f
         + \frac{L_q}{T_s}\,\bigl(i_q^{\star}[k+1]-i_q[k]\bigr)
\end{aligned}
$$


### クロスカップリング電圧のキャンセル

$$
\begin{aligned}
v_d^{\text{decpl}} &= +\omega_e L_q\,i_q \\[4pt]
v_q^{\text{decpl}} &= +\omega_e L_d\,i_d + \omega_e\psi_f
\end{aligned}
$$

これを足したあとは，
1 軸ごとに 1 次遅れ \(G(s)=1/(Ls+R_s)\) となるので
通常の PI で整定できます。

| 観点                  | Dead-beat                | 非干渉化 + PI         |
|-----------------------|--------------------------|-----------------------|
| 目標                  | **1 sample** で誤差ゼロ | 数サンプル以内で十分 |
| 必要モデル            | 離散フルモデル逆演算    | クロス項だけ          |
| 応答速度              | 最速                     | 高帯域だが穏やか      |
| モデル誤差への耐性    | 低                       | 高                    |
| 電圧飽和への耐性      | 低                       | 高                    |
