# dq軸モータモデル (連続時間定式化)

とすると、

* **入力**
  * ${\boldsymbol{v}_{dq}= [\,v_d,\;v_q\,]^{\mathsf T}}$ … 固定子電圧 (d/q軸)
  * ${\omega}$ … 回転子の電気角速度

* **状態量**
  * ${ \boldsymbol{\psi}_{dq}= [\,\psi_d,\;\psi_q\,]^{\mathsf T}}$ … 固定子鎖交磁束

* **出力**
  * ${ \boldsymbol{i}_{dq}= [\,i_d,\;i_q\,]^{\mathsf T}}$ … 相電流
  * ${ T_e }$ … 電磁トルク

ここで、電流は非線形磁化ルックアップテーブル (ID MAP & IQ MAP) から取得されます。

---

## 1. 磁束-電圧ダイナミクス

磁束の時間微分は、印加電圧、抵抗降下、および回転によって生じる逆起電力（90°回転行列 ${ \mathbf{J} }$）の代数和です。

$$
\boxed{\;
\dot{\boldsymbol{\psi}}_{dq}
  = \boldsymbol{v}_{dq}
  \;-\; R_s\,\boldsymbol{i}_{dq}
  \;+\; \omega\,\mathbf{J}\,\boldsymbol{\psi}_{dq}
\;}
\qquad
\mathbf{J}
= \begin{bmatrix} 0 & -1 \\[2pt] 1 & 0 \end{bmatrix}.
$$

---

## 2. 磁化 (逆ルックアップ)

瞬時磁束から、交差飽和を捕捉する2つの独立した*n-D*テーブルから電流が読み取られます。

$$
\boldsymbol{i}_{dq}
=
\begin{bmatrix}
i_d \\ i_q
\end{bmatrix}
=
\begin{bmatrix}
f_d\!\bigl(\psi_d,\psi_q,\lVert\boldsymbol{\psi}_{dq}\rVert\bigr) \\[4pt]
f_q\!\bigl(\psi_d,\psi_q,\lVert\boldsymbol{\psi}_{dq}\rVert\bigr)
\end{bmatrix},
\qquad
\lVert\boldsymbol{\psi}_{dq}\rVert=\sqrt{\psi_d^{2}+\psi_q^{2}}.
$$

*`ID MAP ` は \(f_d\) を返し、*`IQ MAP` は \(f_q\) を返します。*

---

## 3. 電磁トルク

Simulinkのパスは **i** を \(+90^\circ\) 回転させ、**ψ** との内積を取り、通常のクロス積の表現を与えます。

$$
\boxed{\;
T_e
= P_n\bigl(\psi_d\,i_q\;-\;\psi_q\,i_d\bigr)
\;}
$$

ここで、\(P_n\) はブロック *Pn* で使用される（スケーリングされた）極対数です。

---

## 4. 完全なモデルのまとめ

$$
\begin{aligned}
\dot{\boldsymbol{\psi}}_{dq}
    &= \boldsymbol{v}_{dq} - R_s\,\boldsymbol{i}_{dq}
       + \omega\,\mathbf{J}\,\boldsymbol{\psi}_{dq}, \\[6pt]
\boldsymbol{i}_{dq}
    &= \bigl[f_d(\psi_d,\psi_q,\|\boldsymbol{\psi}\|),\;
        f_q(\psi_d,\psi_q,\|\boldsymbol{\psi}\|)\bigr]^{\mathsf T}, \\[6pt]
T_e &= P_n(\psi_d\,i_q - \psi_q\,i_d).
\end{aligned}
$$

この一連の式は、示されているSimulinkダイアグラムが正確に実装している内容です。

* 加算器-積分器ブランチは1行目（磁束ダイナミクス）を実現します。
* ID MAP / IQ MAP テーブルは2行目（非線形磁気モデル）を提供します。
* 回転-ドット積-ゲイン連鎖は3行目（トルク）を計算します。