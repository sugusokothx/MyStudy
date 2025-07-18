## 9.3.1 ベクトル回転器同伴フィルタ
このフィルタは、二相信号を効率的に処理するための基本的な構成です 。

構成: システムの入力と出力にベクトル回転器（信号の位相を回転させるもの）があり、その間に2つの同じ特性を持つフィルタ$F(s)$が並列に配置されています 。入力信号はまず回転され、次に各成分が個別にフィルタリングされ、最後にもう一度逆方向に回転されて出力されます 。
特性: この処理の重要な点は、フィルタ全体の伝達特性が、内部のフィルタ$F(s)$の周波数特性を、シフト信号$ω_0$に応じて周波数軸上でシフトさせたものになることです 。具体的には、入力信号の正相成分に対しては伝達関数が$F(s+jω_0)$となり、逆相成分に対しては$F(s−jω _0)$となります 。これにより、正相成分と逆相成分を分離したり、選択的に処理したりすることが可能になります 。


 ## 9.3.2 D因子の構築

**D因子**は、ベクトル回転器同伴フィルタの機能を数学的に置き換えるために導入された $2 \times 2$ の行列です。

### 定義

D因子 $D(s, \omega_0)$ は、ラプラス演算子 $s$ と外部からのシフト信号 $\omega_0$ を用いて以下のように定義されます：

$$
D(s, \omega_0) = 
\begin{bmatrix}
s & \omega_0 \\
-\omega_0 & s
\end{bmatrix}
= sI + \omega_0 J
$$

### 特性

この D因子は、対角行列 $\operatorname{diag}(s \pm j\omega_0)$ と**相似**の関係にあります。  
これは、D因子がベクトル回転器同伴フィルタと同様の**周波数シフト特性**を持つことを数学的に示しています。

## 9.3 二相信号用 D因子フィルタ

このセクションでは、二相信号（例えばモーター制御で使われる2つの直交信号）を処理するための新しいフィルタ「**D因子フィルタ**」を提案し、その理論と特性を解説しています。  
これは、従来の「**ベクトル回転器同伴フィルタ**」と等価な性能を持ちながら、よりシンプルな構成で実現できるというものです。

---

## 9.3 二相信号用 D因子フィルタ

このセクションでは、二相信号（例えばモーター制御で使われる2つの直交信号）を処理するための新しいフィルタ「**D因子フィルタ**」を提案し、その理論と特性を解説しています。  
これは、従来の「**ベクトル回転器同伴フィルタ**」と等価な性能を持ちながら、よりシンプルな構成で実現できるというものです。

---

### 9.3.1 ベクトル回転器同伴フィルタ

このフィルタは、二相信号を効率的に処理するための基本的な構成です。

#### 構成

- システムの入力と出力に**ベクトル回転器**（信号の位相を回転させるもの）があり、その間に 2つの同じ特性を持つフィルタ $F(s)$ が並列に配置されています。
- 入力信号はまず回転され、次に各成分が個別にフィルタリングされ、最後にもう一度逆方向に回転されて出力されます。

#### 特性

- フィルタ全体の伝達特性は、内部のフィルタ $F(s)$ の周波数特性を、シフト信号 $\omega_0$ に応じて**周波数軸上でシフト**させたものになります。
- 具体的には：
  - **正相成分**に対しては伝達関数が $F(s + j\omega_0)$ となり、
  - **逆相成分**に対しては $F(s - j\omega_0)$ となります。
- これにより、正相成分と逆相成分を**分離**したり、**選択的に処理**したりすることが可能になります。

---

### 9.3.2 D因子の構築

**D因子**は、ベクトル回転器同伴フィルタの機能を**数学的に置き換える**ために導入された $2 \times 2$ の行列です。

#### 定義

D因子 $D(s, \omega_0)$ は、ラプラス演算子 $s$ と外部からのシフト信号 $\omega_0$ を用いて以下のように定義されます：

$$
D(s, \omega_0) = 
\begin{bmatrix}
s & \omega_0 \\
-\omega_0 & s
\end{bmatrix}
= sI + \omega_0 J
$$

#### 特性

- この D因子は、対角行列 $\operatorname{diag}(s \pm j\omega_0)$ と**相似**の関係にあります。
- これは、D因子が**ベクトル回転器同伴フィルタ**と同様の**周波数シフト特性**を持つことを数学的に示しています。

---

### 9.3.3 D因子フィルタの定義と基本特性

**D因子フィルタ**は、スカラフィルタのラプラス演算子 $s$ を **D因子行列**に置き換えることで作られます。

#### 定義

- 通常の 1入力1出力フィルタ $F(s) = \frac{B(s)}{A(s)}$ を基に、その $s$ を $D(s, \omega_0)$ に置き換えて、
  $$ F(D(s, \omega_0)) $$
  として定義されます。

#### 安定性

- D因子フィルタの**安定性**は、元のスカラフィルタ $F(s)$ の安定性と**全く同一**です。
- 元のフィルタが安定であれば、D因子フィルタも自動的に安定性が保証されます。

#### 周波数特性

- ベクトル回転器同伴フィルタと同様に、
  - 入力信号の**正相成分**には $F(s + j\omega_0)$ が作用し、
  - **逆相成分**には $F(s - j\omega_0)$ が作用します。

---


## 9.3 二相信号用 D因子フィルタ

このセクションでは、二相信号（例えばモーター制御で使われる2つの直交信号）を処理するための新しいフィルタ「**D因子フィルタ**」を提案し、その理論と特性を解説しています。  
これは、従来の「**ベクトル回転器同伴フィルタ**」と等価な性能を持ちながら、よりシンプルな構成で実現できるというものです。

---

### 9.3.1 ベクトル回転器同伴フィルタ

このフィルタは、二相信号を効率的に処理するための基本的な構成です。

#### 構成

- システムの入力と出力に**ベクトル回転器**（信号の位相を回転させるもの）があり、その間に 2つの同じ特性を持つフィルタ $F(s)$ が並列に配置されています。
- 入力信号はまず回転され、次に各成分が個別にフィルタリングされ、最後にもう一度逆方向に回転されて出力されます。

#### 特性

- フィルタ全体の伝達特性は、内部のフィルタ $F(s)$ の周波数特性を、シフト信号 $\omega_0$ に応じて**周波数軸上でシフト**させたものになります。
- 具体的には：
  - **正相成分**に対しては伝達関数が $F(s + j\omega_0)$ となり、
  - **逆相成分**に対しては $F(s - j\omega_0)$ となります。
- これにより、正相成分と逆相成分を**分離**したり、**選択的に処理**したりすることが可能になります。

---

### 9.3.2 D因子の構築

**D因子**は、ベクトル回転器同伴フィルタの機能を**数学的に置き換える**ために導入された $2 \times 2$ の行列です。

#### 定義

D因子 $D(s, \omega_0)$ は、ラプラス演算子 $s$ と外部からのシフト信号 $\omega_0$ を用いて以下のように定義されます：

$$
D(s, \omega_0) = 
\begin{bmatrix}
s & \omega_0 \\
-\omega_0 & s
\end{bmatrix}
= sI + \omega_0 J
$$

#### 特性

- この D因子は、対角行列 $\operatorname{diag}(s \pm j\omega_0)$ と**相似**の関係にあります。
- これは、D因子が**ベクトル回転器同伴フィルタ**と同様の**周波数シフト特性**を持つことを数学的に示しています。

---

### 9.3.3 D因子フィルタの定義と基本特性

**D因子フィルタ**は、スカラフィルタのラプラス演算子 $s$ を **D因子行列**に置き換えることで作られます。

#### 定義

- 通常の 1入力1出力フィルタ $F(s) = \frac{B(s)}{A(s)}$ を基に、その $s$ を $D(s, \omega_0)$ に置き換えて、
  $$ F(D(s, \omega_0)) $$
  として定義されます。

#### 安定性

- D因子フィルタの**安定性**は、元のスカラフィルタ $F(s)$ の安定性と**全く同一**です。
- 元のフィルタが安定であれば、D因子フィルタも自動的に安定性が保証されます。

#### 周波数特性

- ベクトル回転器同伴フィルタと同様に、
  - 入力信号の**正相成分**には $F(s + j\omega_0)$ が作用し、
  - **逆相成分**には $F(s - j\omega_0)$ が作用します。

---

### 9.3.4 D因子フィルタの実現

**D因子フィルタ**は、ベクトル回転器同伴フィルタと比較して、**より少ない演算量**で実現できるという大きな利点があります。

#### 構成

- ベクトル回転器同伴フィルタでは、位相を計算するための**積分器**や**三角関数発生器**など、合計 **8個の乗算器**が必要になります。

#### D因子フィルタの利点

- 一方、D因子フィルタは、D因子モジュール1つにつき**スカラ乗算器が2つ**追加で必要になるだけです。
- これにより、特に**低次のフィルタリング処理**において、演算負荷が低く優位性があります。

#### 実現方法

- 具体的な実現法として
  - **「モジュール・ベクトル直接I形」**
  - **「モジュール・ベクトル直接II形」**
- が示されています。

---

