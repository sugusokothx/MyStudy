# パーク変換の基本

通常のパーク変換（d-q変換）は、**静止座標系（α−β座標系）** のベクトルを、**回転する回転子座標系（d-q座標系）** に変換します。この変換には、回転子の電気角 $\theta_e$ が必要です。

---

## 高周波注入法と回転子座標系

高周波注入法では、注入される高周波電圧・電流が回転子の位置推定に利用されます。そのため、**高周波応答成分を回転子座標系**（多くの場合、**γ−δ座標系** と呼ばれる）に変換することで、**回転子の突極性** や **飽和によるインダクタンス変化** を捉えることができます。

---

## パーク変換式

変換式として、高周波電流 $I_{h\alpha} + j I_{h\beta}$ を $I_{h\alpha\beta}$ と定義し、以下のパーク変換で **回転子座標系での高周波電流** $I_{h\gamma\delta}$ を得ます：

$$
I_{h\gamma\delta} = I_{h\alpha\beta} \cdot e^{-j\theta_{h,e}}
$$

ここで、

- $I_{h\alpha\beta} = I_{h\alpha} + j I_{h\beta}$
- $I_{h\gamma\delta} = I_\gamma + j I_\delta$
- $\theta_{h,e}$ は高周波の注入源の基準となる回転子の電気角  
（通常は、推定対象の回転子電気角 $\theta_e$ またはその推定値 $\hat{\theta}_e$ を使用）

---

## 実部・虚部への展開

複素数としての変換結果を、実部と虚部に分解すると以下のようになります：

### 形式1：

$$
I_\gamma = \Re\left(I_{h\alpha\beta} \cdot \left(\cos(-\theta_{h,e}) + j \sin(-\theta_{h,e})\right)\right)
$$

$$
I_\delta = \Im\left(I_{h\alpha\beta} \cdot \left(\cos(-\theta_{h,e}) + j \sin(-\theta_{h,e})\right)\right)
$$

---

### 形式2（展開形）：

$$
I_\gamma = \Re\left((I_{h\alpha} + j I_{h\beta}) \cdot (\cos\theta_{h,e} - j \sin\theta_{h,e})\right)
$$

$$
I_\delta = \Im\left((I_{h\alpha} + j I_{h\beta}) \cdot (\cos\theta_{h,e} - j \sin\theta_{h,e})\right)
$$

これをさらに展開すると：

$$
I_\gamma = I_{h\alpha} \cos\theta_{h,e} + I_{h\beta} \sin\theta_{h,e}
$$

$$
I_\delta = -I_{h\alpha} \sin\theta_{h,e} + I_{h\beta} \cos\theta_{h,e}
$$



数式の由来 高周波電流ベクトルを複素数

$I_{h,\alpha\beta}=I_{h\alpha}+jI_{h\beta}$

で表し，回転子座標系（$\gamma\delta$）に写像するには

$I_{h,\gamma\delta} = I_{h,\alpha\beta}e^{-j\theta_e}$

が用いられる。ここで $\theta_e$ は電気角であり

$e^{-j\theta_e}=\cos\theta_e-j\sin\theta_e$。
複素積を展開すると

$$
\begin{aligned}
I_{h,\gamma\delta}
&=(I_{h\alpha}+jI_{h\beta})(\cos\theta_e-j\sin\theta_e) \\
&=\bigl(I_{h\alpha}\cos\theta_e+I_{h\beta}\sin\theta_e\bigr)
 \;+\;j\bigl(-I_{h\alpha}\sin\theta_e+I_{h\beta}\cos\theta_e\bigr).
\end{aligned}
$$

よって

$\gamma$ 軸成分（実部）

$I_\gamma = I_{h\alpha}\cos\theta_e + I_{h\beta}\sin\theta_e$
$\delta$ 軸成分（虚部）

$I_\delta = -I_{h\alpha}\sin\theta_e + I_{h\beta}\cos\theta_e$
