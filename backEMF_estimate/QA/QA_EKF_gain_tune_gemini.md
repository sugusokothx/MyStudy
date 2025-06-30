

# **永久磁石同期モータの拡張カルマンフィルタ（EKF）に基づくパラメータ推定に関する診断および戦略的ガイド**

## **第1部：EKFに基づくパラメータ推定の基礎原理**

提示されたコードにおける拡張カルマンフィルタ（EKF）の推定性能が期待通りに機能しない根本原因を理解するためには、まずEKFの調整という表層的な問題から、その根底にある可観測性（Observability）および持続的励振（Persistent Excitation）という、より本質的な概念へと理論的基盤を移行させる必要があります。本章では、この問題の核心をなすこれらの理論的背景を体系的に解説します。

### **1.1. 状態とパラメータの同時推定器としてのEKF**

まず、この問題を正しく位置づけることが不可欠です。提示された課題は、単にノイズの多い信号をフィルタリングすることではなく、**システム同定（System Identification）**、すなわちシステムの未知の特性を明らかにすることにあります。これは、状態ベクトルxに未知パラメータ（dphi\_d, dphi\_q, dpsi\_f, R\_s）を追加（拡張）することで実現されます。

#### **「ランダムウォーク」モデルの仮定**

EKFのプロセスモデル\_f\_continuousでは、拡張されたパラメータの時間微分d/dtがゼロであると仮定されています。

Python

\# \_f\_continuous内の抜粋  
return np.array(\[di\_d, di\_q, 0.0, 0.0, 0.0, 0.0\])

これは、パラメータを「プロセスノイズにさらされる定数」として扱う手法であり、「ランダムウォーク」モデルとして知られています。このアプローチは一般的かつ実用的な簡略化ですが、フィルタの挙動と調整に重大な影響を及ぼします 。このモデルにおいて、パラメータに対応するプロセスノイズ共分散Qの要素は、もはや物理的な外乱ではなく、パラメータの**定常性に対する不確かさ**や、その期待される時間変化率を表現するものとなります。

#### **示唆されること**

この状態拡張により、EKFの性能は、単に動的な状態を追従する能力だけでなく、観測された出力から隠れたパラメータを推測するという逆問題を解くための情報が十分に得られるかどうかに依存するようになります。この視点の転換が、次節以降で詳述する可観測性の議論への導入となります。

### **1.2. 調整レバーとしての共分散行列（QおよびR）：技巧から科学へ**

EKFの調整は、しばしば「ブラックアート（黒魔術）」と表現されます 。ここでの目標は、このプロセスを物理的原理に根ざして解明し、体系的なアプローチを確立することです。

#### **測定ノイズ共分散（R）**

* **定義と役割**: Rは測定ノイズv\_kの共分散を表し、フィルタが測定値（ノイズが付加されたi\_d\_true, i\_q\_true）をどれだけ「信頼しないか」を定量化します 3。  
  Rの値が大きいほど、フィルタはセンサがノイズが多いと判断し、自身の内部モデルによる予測値をより重視するようになります 2。  
* **実践的な設定**: Rは、共分散行列の中で最も直感的に設定が可能です。その対角要素は、対応するセンサノイズの分散（標準偏差の2乗、σ²）に設定されるべきです 5。提示されたコードでは  
  obs\_noise\_std \= np.sqrt(np.diag(ekf.R))としてノイズを*生成*していますが、これは循環論法です。実際の応用では、モータ停止時などにセンサノイズを実測し、その統計的性質からRを決定します 8。シミュレーションにおける  
  R\_diag=(1e-4, 1e-4)という設定は、ノイズの標準偏差がsqrt(1e-4) \= 0.01 Aに相当し、妥当な初期値と言えます。

#### **プロセスノイズ共分散（Q）**

* **定義と役割**: QはRよりもはるかに抽象的な行列です。これはプロセスノイズw\_kの共分散を表し、主に二つの要因を内包します。1) 状態に影響を与える実際のランダムな外乱、そして 2\) **モデル化されていないダイナミクスやモデルの簡略化に伴う誤差**です 1。  
  Qの値が大きいほど、フィルタは自身のモデルが不正確である、あるいはモデルが捉えきれない形で状態が変化していると判断し、測定値により大きな重みを与えるようになります 1。  
* **提示されたQの解剖**: 状態ベクトルはx \=です。Qの対角要素は、各状態における1ステップあたりのノイズの期待分散に対応します。  
  * Q\_diag, Q\_diag（i\_d, i\_q用）: これらの項は、電圧方程式における誤差、例えば線形化誤差、無視された交差結合効果 12、そしてEKF内の線形インダクタンス  
    Ld, LqとプラントモデルのLUT（ルックアップテーブル）ベースのインダクタンスとの間の不一致などを吸収します。  
  * Q\_diag, Q\_diag（dphi\_d, dphi\_q用）: これらは磁束マップモデル自体の不確かさを表します。1e-8という値は磁束マップへの高い信頼を示唆していますが、今回の問題の主眼ではありません。  
  * Q\_diag（dpsi\_f用）: **これは極めて重要です**。この項は、dpsi\_fの1ステップあたりの**変化**の期待分散を表します。これは、温度による永久磁石磁束のドリフト速度をモデル化します。1e-9という値は非常に小さく、フィルタに対してdpsi\_fがほぼ完全に一定であると伝えています。  
  * Q\_diag（R\_s用）: **これも同様に重要です**。この項は、R\_sの1ステップあたりの**変化**の期待分散を表し、その熱的ドリフトをモデル化します。1e-9という値は、こちらも非常に小さい設定です。

#### **Q/R比とフィルタ帯域幅**

フィルタの挙動を深く理解するためには、QとRの相対的な関係、すなわちQ/R比を考察する必要があります。

1. カルマンゲインKは、P H^T (H P H^T \+ R)^-1に比例します 13。状態更新は  
   x\_hat \+= K \* y（yは残差）で行われます。  
2. 事後誤差共分散Pは、予測ステップでP \= F P F^T \+ Qと更新されます。  
3. もしQがRに対して大きい場合、Pは予測ステップで急速に増大し、結果としてカルマンゲインKも大きくなります。大きなKは、測定残差yによる補正量K \* yが大きくなることを意味し、フィルタは測定値に敏感に追従します。これは「広帯域」なフィルタと解釈できます。  
4. 逆に、QがRに対して小さい場合、Pの増大は抑制され、Kは小さくなります。フィルタは測定値をあまり信用せず、自身のモデルによる予測を重視します。これは「狭帯域」なフィルタです。

提示されたコードでは、パラメータに関するQの値（1e-9）が測定ノイズに関連するRの値（1e-4）に対して極端に小さく設定されています。これにより、パラメータ推定のためのフィルタは極めて「狭帯域」となり、測定値に含まれるdpsi\_fやR\_sに関する情報に対して事実上「耳を貸さない」状態になっています。これは調整上の問題ですが、次節で述べる、より根本的な可観測性の問題に比べれば二次的な要因です。

### **1.3. システム可観測性の決定的概念**

* **定義**: 可観測性とは、ある期間にわたるシステムの外部入力と出力の知識から、その内部状態を一意に推定できるかどうかを決定するシステム固有の特性です 14。線形システムでは、可観測性行列のランク条件によってこれを判定できます。PMSMのような非線形システムでは、局所弱可観測性といったより複雑な概念が用いられますが、その基本原理は同じです 14。  
* **核心的な診断上の問い**: ユーザーの直面している問題は、次のように再定義できます。「一定の入力（v\_d, v\_q, ω\_e）と測定出力（i\_d, i\_q）が与えられたとき、このシステムはパラメータdpsi\_fとR\_sに関して可観測であるか？」  
* **不可避の結論**: 第2部で詳述するように、この問いに対する答えは「否」です。この特定の運転条件下では、システムは可観測ではなく、良くても極めて弱い可観測性しか持ちません。R\_sの変化がもたらす影響とdpsi\_fの変化がもたらす影響は、出力電流上では曖昧であり、互いに模倣し合うことができます。これはEKFアルゴリズムの欠陥ではなく、**実験設計そのものに内在する構造的な問題**です。多くの研究が、複数パラメータの同時推定は特定の条件下でのみ可能であることを指摘しています 14。

### **1.4. 持続的励振：可観測性を確保する鍵**

* **定義**: 持続的励振（Persistent Excitation, PE）とは、システムのパラメータを可観測にするために入力信号に課される条件です 17。入力信号がシステムの全ての動的モードを励起するのに十分な周波数成分を「豊富に」含んでいる場合、その信号は持続的に励振していると言えます。これにより、パラメータ同定に必要な十分な情報が提供されます 17。  
* **一定入力が失敗する理由**: 一定の入力信号は、直流（DC）成分、すなわちゼロ周波数成分しか持ちません。これは持続的に励振する信号の対極にあります 17。ユーザーが  
  u\_cmdを定数ベクトルとして選択したことが、推定が失敗する根本的な原因です。  
* **必要不可欠な解決策**: パラメータdpsi\_fとR\_sを可観測にするためには、入力ベクトルu\_cmdが**時間的に変化**しなければなりません。これは、制御入力に特定の試験信号（パルス、正弦波、PRBSなど）を意図的に注入することで達成されます 15。  
* PEは万能薬ではなく、診断ツールである:  
  PEの概念は、単に「ノイズを加えればよい」というものではありません。それは、隠れたパラメータを可視化する方法でシステムを探査する、情報豊富な特定の入力信号を設計することです。この探査の欠如が、ユーザーの問題の根源です。  
  この洞察を深めるために、PMSMの電圧方程式を考察します 12。

  d軸電圧方程式: vd​=Rs​id​+dtdλd​​−ωe​λq​  
  q軸電圧方程式: vq​=Rs​iq​+dtdλq​​+ωe​λd​  
  ここで、永久磁石磁束ψ\_fは、主にd軸磁束λ\_dの項（λd​=Ld​id​+ψf​）に現れます。したがって、ψ\_fに関する情報を引き出すためには、d軸のダイナミクスを刺激する必要があります。これは、最も効果的な励振信号がv\_dの変動であることを意味します。v\_dを変化させるとi\_dに応答が引き起こされますが、この応答はλ\_d、ひいてはψ\_fに固有に依存します。これにより、フィルタはψ\_fの影響を、両方の方程式に現れるR\_sの影響から分離することが可能になります。  
  近年の研究では、PE条件なしで推定を達成する高度なアルゴリズムも探求されていますが、それらは過去のデータを蓄積・利用することに依存しており、標準的な手法にとってPEがいかに基本的な要件であるかを逆説的に示しています 20。

---

## **第2部：実装コードの詳細分析**

本章では、第1部で確立した理論をユーザーの提供したコードに直接結びつけ、失敗のメカニズムを具体的に診断します。

### **2.1. EKFの内部プロセスモデル（\_f\_continuous）の批評**

#### **モデル構造**

関数\_f\_continuousは、d-q回転座標系における標準的なPMSMの電圧方程式を実装しています。

Python

def \_f\_continuous(self, x, u):  
    i\_d, i\_q, dphi\_d, dphi\_q, dpsi\_f, R\_s \= x  
    v\_d, v\_q, ω\_e \= u

    \# EKF内部のモデル  
    λ\_d \= self.Ld \* i\_d \+ self.psi\_f\_nom \+ dpsi\_f \+ dphi\_d  
    λ\_q \= self.Lq \* i\_q \+ dphi\_q

    di\_d \= (v\_d \+ ω\_e \* λ\_q \- R\_s \* i\_d) / self.Ld  
    di\_q \= (v\_q \- ω\_e \* λ\_d \- R\_s \* i\_q) / self.Lq  
    return np.array(\[di\_d, di\_q, 0.0, 0.0, 0.0, 0.0\])

#### **主要な簡略化とその影響**

* **線形インダクタンス**: EKFは定数LdとLqを使用しています。一方、「現実世界」を模したMotorModelはルックアップテーブル（phi\_d\_lut, phi\_q\_lut）を使用しており、これは真のインダクタンスがi\_dとi\_qの関数（磁気飽和）であることを示唆しています 12。この不一致はモデル化誤差の一因となり、プロセスノイズ共分散  
  Qによって吸収されるべきです。運転点が大きく変化すると、この誤差は無視できなくなり、性能を低下させる可能性があります。  
* **パラメータのダイナミクス**: 前述の通り、モデルはd(dpsi\_f)/dt \= 0およびd(R\_s)/dt \= 0と仮定しています。このモデルは、パラメータがフィルタの更新周期に比べて十分にゆっくりと変化する場合にのみ有効です。シミュレーションにおける真値dpsi\_f\_trueは緩やかなランプ状に変化しており、このモデルと整合性があります。しかし、フィルタがこのランプ変化を追従する能力は、対応するQの項と、後述する可観測性によって支配されます。

### **2.2. 可観測性の失敗の診断：直感的および数学的視点**

#### **定常状態における曖昧さ**

システムが定常状態にあり、電流の時間微分di/dtがほぼゼロであると仮定して考察します。このとき、EKFのモデル方程式は次のように簡略化されます。

vd​≈Rs​id​−ωe​(Lq​iq​)  
vq​≈Rs​iq​+ωe​(Ld​id​+ψf\_nom​+dψf​)

#### **問題点**

q軸電圧方程式に注目してください。R\_s \* i\_qの項とω\_e \* dpsi\_fの項が加算されています。ここで、真のR\_sがΔR\_sだけ増加し、同時に真のdpsi\_fがΔψ\_fだけ減少したと想像してください。フィルタはi\_qの変化を観測しますが、この変化をどちらのパラメータの変動に帰属させるべきか判断できません。

#### **数学的な曖昧さ**

フィルタは測定値と予測値の差である残差y \= z \- H@x\_hatを観測します。この残差は状態推定値の誤差によって駆動されます。もし、R\_sの推定誤差（正の値ΔR\_s\_est）とdpsi\_fの推定誤差（負の値Δψ\_f\_est）が組み合わさって、予測電流に同様の影響を与えることができる場合、フィルタは両者を区別できません。具体的には、ΔR\_s\_est \* i\_qと-ω\_e \* Δψ\_f\_estがほぼ等しい場合（ΔR\_s\_est \* i\_q ≈ \-ω\_e \* Δψ\_f\_est）、q軸電圧方程式への正味の影響はほぼゼロとなり、フィルタの推定は停滞します。入力が一定であるため、i\_qとω\_eも比較的一定となり、この代数的な曖昧さが持続的に存在することになります。

この状況は、システムのパラメータ推定に関するランクが不足している状態（rank-deficient）として知られています 16。文献は、信号注入なしではPMSMのパラメータは最大でも2つしか同時推定できず、その中でも(

R\_s, ψ\_f)の組み合わせは特に困難であることを裏付けています 14。

### **2.3. 初期共分散の原則に基づいた再評価**

#### **初期Pの批評**

初期誤差共分散Pは、状態推定値の初期不確かさを表します。提示されたコードではP \= np.diag(\[1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4\])と設定されています。パラメータに対応する値（1e-4）は比較的小さく、初期値に対する高い信頼度を示唆しています。一般的な実践としては、未知のパラメータに対してはPの初期値を大きく設定し、初期の収束を速めることが推奨されます 22。例えば、

PとPを1.0以上に設定することが考えられます。

#### **初期Qの批評**

1.2節で分析したように、パラメータに関連するQの値（1e-9）は、おそらく小さすぎます。これは、1ステップあたりの変化の標準偏差がsqrt(1e-9) ≈ 3e-5であることを意味します。しかし、この評価はより深い分析を必要とします。

シミュレーションにおける真のdpsi\_fは、1ステップあたり0.10 \* psi\_f\_nom / 10000 \= 0.1 \* 0.08 / 10000 \= 8e-7だけ変化します。ランダムウォークモデルのプロセスノイズの分散は、この変化量の2乗、すなわち(8e-7)^2 ≈ 6.4e-13程度であると期待されます。この観点から見ると、ユーザーが設定したQの値1e-9は、実際のプロセス分散よりもはるかに大きく、先の「小さすぎる」という評価と矛盾するように見えます。

この矛盾を解決する鍵は、EKFが決定論的なランプ変化をどのように扱うかを理解することにあります。

1. 真のdpsi\_fは、確率的なランダムウォークではなく、決定論的なランプ（一定の傾きを持つ直線的な変化）です。  
2. EKFのプロセスモデルd(dpsi\_f)/dt \= 0には、このランプ変化を説明する項が存在しません。  
3. そのため、EKFはこのモデルと現実の間の体系的なズレ（ランプ変化）を、確率的なプロセスノイズw\_kとして解釈しようと試みます。  
4. フィルタがこのランプ変化に追従するためには、プロセスノイズQが、カルマンゲインがゼロに収束してしまうのを防ぐのに十分な大きさでなければなりません。つまり、フィルタに対して「d/dt \= 0というモデルは常に間違っている」と教える必要があります。

結論として、ユーザーのQの値1e-9は絶対的に小さすぎるわけではないかもしれませんが、**真の問題はQのチューニングそのものではありません**。たとえQが完璧に調整されたとしても、持続的励振の欠如により測定値から必要な情報が得られないため、フィルタはランプ変化を追従できません。R\_sとdpsi\_fの間の曖昧さが、フィルタへの補正信号を汚染し、正しい方向への更新を妨げます。したがって、Qの調整は二次的な問題であり、まず可観測性の問題を解決しなければなりません。現在の状態でQを調整しようとするのは、電波の届かない部屋でラジオをチューニングしようとするようなものです。

---

## **第3部：収束達成のための戦略的ロードマップ**

本章では、推定問題を解決するための明確かつ実行可能な段階的計画を提示します。戦略は、最も単純なものから最も効果的なものへと順序立てられています。

### **3.1. 戦略1：診断とベースライン調整のための分離推定**

* **目的**: 問題を単純化し、変数を分離してQのベースライン調整を行う。これは極めて重要なデバッグステップです。  
* **手順**:  
  1. **R\_sのみを推定する**: EKFを修正し、dpsi\_fを真値（未知の場合はゼロ）に固定します。具体的には、dpsi\_fの推定を無効化するため、対応するQの項（Q）を極めて小さな値（例：1e-20）に設定し、更新されないようにします。その上で、Rs\_estがRs\_trueに収束するまでQを調整します。比較的大きな値、例えば1e-6から始め、挙動を見ながら調整します。  
  2. **dpsi\_fのみを推定する**: 逆の操作を行います。R\_sを真値に固定し、その推定を無効化します（Q \= 1e-20）。次に、dpsi\_f\_estがdpsi\_f\_trueに収束するまでQを調整します。これにより、熱的ドリフトを追従するために必要なプロセスノイズのベースライン値が得られます。  
* **期待される結果**: このプロセスでも、持続的励振が欠如しているため性能は依然として低い可能性がありますが、次の、より重要なステップに進むための適切なスケールのQ値が得られます。

### **3.2. 戦略2：持続的励振信号の注入（核心的解決策）**

* **目的**: 入力信号を「十分に豊富」にすることで、システムパラメータを可観測にする 17。  
* **実装**: 定数ベクトルu\_cmdを時間的に変化するものに置き換える必要があります。d軸のダイナミクスが磁束に最も直接的に関連するため、d軸電圧指令v\_d\_cmdに信号を注入します。

#### **手法A：擬似ランダムバイナリシーケンス（PRBS）の注入**

* **PRBSを選択する理由**: PRBSはシステム同定において優れた選択肢です。決定論的でありながら、そのスペクトルは広帯域でホワイトノイズに似ており、実装も容易です 18。  
* **コード例**:  
  Python  
  \# メインループ内  
  prbs\_amplitude \= 5.0  \# \[V\]、この値を調整する  
  prbs\_signal \= 0.0  
  if k % 1000 \== 0:  \# 100 ms ごとに状態を変化させる  
      \# 前回の値を保持するための変数をループ外で定義する必要がある  
      \# prbs\_state \= prbs\_amplitude \* (2 \* np.random.randint(0, 2\) \- 1\)  
      \# この例では、ループ内で毎回ランダムに生成する  
      prbs\_signal \= prbs\_amplitude \* (2 \* np.random.randint(0, 2) \- 1)

  v\_d\_cmd\_excited \= v\_d\_cmd \+ prbs\_signal  
  u\_cmd \= (v\_d\_cmd\_excited, v\_q\_cmd, ω\_e)

  *注: 上記コードはPRBSの単純な実装例です。実用上は、ループ外で状態を保持する変数を定義し、周期的に更新する方が適切です。*

#### **手法B：正弦波の注入**

* **正弦波を選択する理由**: 特定の周波数の正弦波を注入することでも励振を与えることができます。PRBSほど周波数成分は「豊富」ではありませんが、その周波数がシステムの動的応答範囲内にあり、かつ電気角周波数ω\_eから十分に離れていれば効果的です。  
* **コード例**:  
  Python  
  \# メインループ内  
  sine\_amplitude \= 5.0  \# \[V\]  
  sine\_frequency \= 10 \* 2 \* np.pi  \# 10 Hz の注入周波数 \[rad/s\]  
  v\_d\_cmd\_excited \= v\_d\_cmd \+ sine\_amplitude \* np.sin(sine\_frequency \* k \* DT)  
  u\_cmd \= (v\_d\_cmd\_excited, v\_q\_cmd, ω\_e)

* **理論的根拠の再確認**: v\_dを変動させることで、i\_dの変化を強制します。v\_dの変化とi\_dの応答との関係は、d軸のダイナミクス、特にψ\_d \= L\_d i\_d \+ ψ\_fによって支配されます。これがフィルタにψ\_fの固有の「署名」を提供し、より一般的な影響を持つR\_sからψ\_fを区別することを可能にします。これは、文献で議論されているランク不足の問題に直接対処するものです 16。

### **3.3. 体系的なチューニング方法論（励振導入後）**

* **目的**: 持続的励振が有効になった状態で、性能を最適化するために共分散行列の最終的かつ体系的な調整を行います。  
* **段階的手順**:  
  1. **Rの設定**: Rが既知の測定ノイズ分散に基づいていることを確認します。R\_diag \= \[std\_dev\_i\*\*2, std\_dev\_i\*\*2\]。  
  2. **状態（i\_d, i\_q）に関するQの調整**: パラメータを固定した状態で（あるいは分離推定の値を用いて）、Q\_diagとQ\_diagを調整します。i\_d\_estとi\_q\_estのプロットを観察します。もし推定値がノイズ過多で測定ノイズに追従しすぎている場合、Rが小さすぎるかQが大きすぎます。もし推定値が滑らかすぎるが真のダイナミクスに遅れる場合、Rが大きすぎるかQが小さすぎます 6。推定値が滑らかでありながら応答性が良いバランス点を見つけます。  
  3. **パラメータ（dpsi\_f, R\_s）に関するQの調整**: ここで同時推定を有効にします。戦略3.1で得たベースライン値を初期値として使用します。  
     * パラメータ推定値の収束が遅すぎる場合は、QとQを大きくします。  
     * 推定値がノイズ過多で激しく変動する場合は、これらの値を小さくします。  
     * 目標は、フィルタが真のパラメータのランプ変化を追従できる範囲で、かつ推定値に過度なノイズを導入しない、最小のQ値を見つけることです。

#### **提案：EKF共分散チューニングガイド**

この表は、アドホックな調整プロセスを、構造化され再現可能なエンジニアリングタスクへと転換するものです。これは、EKF実装における一般的な難点を直接的に解決することを目的としています 2。抽象的な行列の値を、物理的な意味と観測可能な挙動に結びつけます。

| 状態/パラメータ | 共分散 | ノイズの物理的意味 | 調整目標 | 値が大きすぎる場合 | 値が小さすぎる場合 | 初期提案値（PE導入後） |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| i\_d, i\_q | Q, Q | 電圧モデルの不確かさ（線形化誤差、VSI非線形性、Ld/Lqの不一致） | 電流推定の応答性とノイズ除去のバランスを取る。 | 推定値がノイズ過多になり、測定ノイズに過剰に追従する。 | 推定値が鈍くなり、真の電流ダイナミクスに遅れる。 | 1e-5 ～ 1e-7 |
| dpsi\_f | Q | 温度による磁束の期待変化率。ランダムウォークモデルの不確かさ。 | ノイズを伴わずにdpsi\_fの緩やかなドリフトを追従する。 | 推定値がノイズ過多になり、電流の変動に過敏に反応する。 | 推定値の収束が遅すぎる、またはランプ変化を追従できない。 | 1e-8 ～ 1e-10 |
| R\_s | Q | 温度による抵抗の期待変化率。 | ノイズを伴わずにR\_sの緩やかなドリフトを追従する。 | 推定値がノイズ過多で不安定になる。 | 推定値が「停滞」し、収束しない。 | 1e-7 ～ 1e-9 |
| i\_d\_meas, i\_q\_meas | R, R | 電流センサノイズの分散（σ²）。 | 物理的なセンサの品質を正確に表現する。 | フィルタがモデルを過信し、応答が鈍くなる。 | フィルタが測定値を過信し、推定値がノイズ過多になる。 | (センサ標準偏差)² 例：(0.01)²=1e-4 |

---

## **第4部：高度な考察とロバスト性への道筋**

最終章では、現在のアプローチの限界について文脈を提供し、さらなる改善への道筋を示すことで、本報告書の専門性を確固たるものにします。

### **4.1. モデル忠実度と線形化誤差**

#### **LUTの影響**

プラントモデルが磁束の計算にルックアップテーブル（LUT）を使用していること（phi\_d\_lut, phi\_q\_lut）は、特に高負荷時の磁気飽和領域において、顕著な非線形性を導入します 12。EKFの線形モデル（

Ld\*i\_d, Lq\*i\_q）は、この非線形性に対する一次のテイラー展開近似に過ぎません。この線形化から生じる誤差は、プロセスノイズQが吸収すべき主要な要素となります。注入された励振信号がモータを異なる飽和領域に移行させる場合、この誤差は一定ではなくなり、EKFの性能に挑戦を突きつける可能性があります。

#### **改善の可能性**

より高度なフィルタでは、LUTのヤコビアン（偏微分行列）をEKFのF行列の計算に直接組み込むことで、より正確な線形化を実現できます。ただし、これにより実装の複雑さは増大します。

### **4.2. 代替的な推定アーキテクチャ**

#### **文脈**

適切に調整されたEKFと持続的励振があれば、ユーザーの問題は解決するはずですが、特に複数パラメータの同時推定という困難な課題に対しては、他の手法が存在することを知っておくことは有益です。

* **モデル規範形適応システム（MRAS）**: MRASはパラメータ推定において一般的な代替手法です。これは二つのモデル、すなわちパラメータに依存しない「規範モデル」と、パラメータに依存する「可調整モデル」を使用します。適応則が、パラメータ推定値を調整することによって両モデル間の誤差をゼロに駆動します 24。一部のMRAS構造は、  
  R\_sとR\_r（または磁束）の同時推定のために特別に設計されています 24。  
* **デュアル/マルチEKF**: このアプローチは、相互作用する複数のEKFを使用します。例えば、一つのEKFが高速に変化する状態（電流）を推定し、もう一つのより低速なEKFが低速に変化するパラメータ（抵抗、磁束）を推定します。これにより、安定性と調整の容易さが向上する場合があります。

#### **意義**

これらの手法に言及することは、この分野における包括的な理解を示すとともに、ユーザーが将来、さらにロバストなシステムを構築する必要が生じた場合に、探求すべきキーワードと概念を提供するものです。

---

## **結論と推奨事項**

#### **核心的診断**

EKFが収束に失敗した原因は、単純なチューニングの問題ではなく、**システムの可観測性**という根本的な問題にあります。一定の電圧と速度指令を用いる現在の運転方法は**持続的励振**を提供せず、その結果、フィルタが固定子抵抗（R\_s）の変動と永久磁石磁束（dpsi\_f）の変化の影響を区別することを不可能にしています。

#### **優先されるべき行動計画**

1. **持続的励振の導入**: これは最も重要なステップです。v\_d\_cmd指令に時間的に変化する信号（PRBSを推奨）を注入するようにメインループを修正してください。これは収束を達成するために譲れない必須条件です。  
2. **体系的な共分散チューニング**: 持続的励振を導入した後、第3部で概説した体系的なチューニング手順に従ってください。まずRを設定し、次に状態に関するQの項、最後にパラメータに関するQの項を、提供されたガイド表を参考に調整します。  
3. **初期パラメータ不確かさの増大**: パラメータに対する初期誤差共分散行列Pの対応する要素（P, P）の値を大きく設定し、初期収束を促進してください。  
4. **（任意）分離診断の実施**: 同時推定に取り組む前に、分離推定（戦略3.1）を実行し、R\_sとdpsi\_fそれぞれに対するQのベースライン値を見つけることを検討してください。これは、同時推定時のチューニングの出発点として非常に有効です。

#### **引用文献**

1. matlab \- Question About $ Q $ Matrix (Model Process Covariance) in Kalman Filter, 6月 24, 2025にアクセス、 [https://dsp.stackexchange.com/questions/21796/question-about-q-matrix-model-process-covariance-in-kalman-filter](https://dsp.stackexchange.com/questions/21796/question-about-q-matrix-model-process-covariance-in-kalman-filter)  
2. Kalman filter error growth \- Orekit Forum, 6月 24, 2025にアクセス、 [https://forum.orekit.org/t/kalman-filter-error-growth/1699](https://forum.orekit.org/t/kalman-filter-error-growth/1699)  
3. Extended Kalman filter \- Wikipedia, 6月 24, 2025にアクセス、 [https://en.wikipedia.org/wiki/Extended\_Kalman\_filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)  
4. Extended Kalman Filter — AHRS 0.4.0 documentation, 6月 24, 2025にアクセス、 [https://ahrs.readthedocs.io/en/latest/filters/ekf.html](https://ahrs.readthedocs.io/en/latest/filters/ekf.html)  
5. Kalman filter \- understanding the noise covariance matrix, 6月 24, 2025にアクセス、 [https://dsp.stackexchange.com/questions/23776/kalman-filter-understanding-the-noise-covariance-matrix](https://dsp.stackexchange.com/questions/23776/kalman-filter-understanding-the-noise-covariance-matrix)  
6. How Do You Determine the R and Q Matrices of a Kalman Filter? : r/ControlTheory \- Reddit, 6月 24, 2025にアクセス、 [https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how\_do\_you\_determine\_the\_r\_and\_q\_matrices\_of\_a/](https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how_do_you_determine_the_r_and_q_matrices_of_a/)  
7. The Extended Kalman Filter: An Interactive Tutorial for Non-Experts, 6月 24, 2025にアクセス、 [https://simondlevy.github.io/ekf-tutorial/kalman\_14.html](https://simondlevy.github.io/ekf-tutorial/kalman_14.html)  
8. How do we determine noise covariance matrices Q & R? \- ResearchGate, 6月 24, 2025にアクセス、 [https://www.researchgate.net/post/How\_do\_we\_determine\_noise\_covariance\_matrices\_Q\_R](https://www.researchgate.net/post/How_do_we_determine_noise_covariance_matrices_Q_R)  
9. How to tune EKF for attitude estimation? \- Robotics \- Arduino Forum, 6月 24, 2025にアクセス、 [https://forum.arduino.cc/t/how-to-tune-ekf-for-attitude-estimation/1119295](https://forum.arduino.cc/t/how-to-tune-ekf-for-attitude-estimation/1119295)  
10. Kalman filter in computer vision: the choice of Q and R noise covariances \- Stack Overflow, 6月 24, 2025にアクセス、 [https://stackoverflow.com/questions/21245167/kalman-filter-in-computer-vision-the-choice-of-q-and-r-noise-covariances](https://stackoverflow.com/questions/21245167/kalman-filter-in-computer-vision-the-choice-of-q-and-r-noise-covariances)  
11. Kalman Filter \- Implementation, Parameters and Tuning \- Signal Processing Stack Exchange, 6月 24, 2025にアクセス、 [https://dsp.stackexchange.com/questions/3039/kalman-filter-implementation-parameters-and-tuning](https://dsp.stackexchange.com/questions/3039/kalman-filter-implementation-parameters-and-tuning)  
12. Online parameter estimation for permanent magnet synchronous machines : an overview, 6月 24, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf](https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf)  
13. Extended Kalman Filter Basics: A Practical Deep Dive, 6月 24, 2025にアクセス、 [https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide](https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide)  
14. Online multi-parameter estimation of interior permanent magnet motor drives with finite control set model predictive control \- MPLab, 6月 24, 2025にアクセス、 [https://mplab.ee.columbia.edu/sites/default/files/content/Publications/Nalakath2016EPA.pdf](https://mplab.ee.columbia.edu/sites/default/files/content/Publications/Nalakath2016EPA.pdf)  
15. Parameter Estimation for Condition Monitoring of PMSM Stator Winding and Rotor Permanent Magnets | Request PDF \- ResearchGate, 6月 24, 2025にアクセス、 [https://www.researchgate.net/publication/260542124\_Parameter\_Estimation\_for\_Condition\_Monitoring\_of\_PMSM\_Stator\_Winding\_and\_Rotor\_Permanent\_Magnets](https://www.researchgate.net/publication/260542124_Parameter_Estimation_for_Condition_Monitoring_of_PMSM_Stator_Winding_and_Rotor_Permanent_Magnets)  
16. A Fast Online Full Parameter Estimation of a PMSM with Sinusoidal Signal Injection \- RWTH Aachen University, 6月 24, 2025にアクセス、 [http://bib.iem.rwth-aachen.de/IEMpublications/AltesBib/2015QLAFast.pdf](http://bib.iem.rwth-aachen.de/IEMpublications/AltesBib/2015QLAFast.pdf)  
17. Persistent excitation conditions and their implications | Adaptive and Self-Tuning Control Class Notes | Fiveable, 6月 24, 2025にアクセス、 [https://library.fiveable.me/adaptive-and-self-tuning-control/unit-8/persistent-excitation-conditions-implications/study-guide/fxVG89qNGAu0wkQI](https://library.fiveable.me/adaptive-and-self-tuning-control/unit-8/persistent-excitation-conditions-implications/study-guide/fxVG89qNGAu0wkQI)  
18. Methodology to estimate parameters of an excitation system based on experimental conditions | Request PDF \- ResearchGate, 6月 24, 2025にアクセス、 [https://www.researchgate.net/publication/235617656\_Methodology\_to\_estimate\_parameters\_of\_an\_excitation\_system\_based\_on\_experimental\_conditions](https://www.researchgate.net/publication/235617656_Methodology_to_estimate_parameters_of_an_excitation_system_based_on_experimental_conditions)  
19. PMSM \- Permanent magnet synchronous motor with sinusoidal flux distribution \- MATLAB, 6月 24, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsm.html](https://www.mathworks.com/help/sps/ref/pmsm.html)  
20. Online System Identification Algorithm without Persistent Excitation for Robotic Systems: Application to Reconfigurable Autonomous Vessels, 6月 24, 2025にアクセス、 [https://senseable.mit.edu/papers/pdf/20191104\_Kayakan-etal\_OnlineSystem\_IROS.pdf](https://senseable.mit.edu/papers/pdf/20191104_Kayakan-etal_OnlineSystem_IROS.pdf)  
21. \[2106.08773\] Persistent Excitation is Unnecessary for On-line Exponential Parameter Estimation: A New Algorithm that Overcomes this Obstacle \- arXiv, 6月 24, 2025にアクセス、 [https://arxiv.org/abs/2106.08773](https://arxiv.org/abs/2106.08773)  
22. How to calculate the initial matrix P\_0, Q\_0, R\_0 for an extended Kalman filter?, 6月 24, 2025にアクセス、 [https://www.researchgate.net/post/How\_to\_calculate\_the\_initial\_matrix\_P\_0\_Q\_0\_R\_0\_for\_an\_extended\_Kalman\_filter](https://www.researchgate.net/post/How_to_calculate_the_initial_matrix_P_0_Q_0_R_0_for_an_extended_Kalman_filter)  
23. Tuning of Extended Kalman Filter for nonlinear State Estimation \- IOSR Journal, 6月 24, 2025にアクセス、 [https://www.iosrjournals.org/iosr-jce/papers/Vol18-issue5/Version-4/C1805041419.pdf](https://www.iosrjournals.org/iosr-jce/papers/Vol18-issue5/Version-4/C1805041419.pdf)  
24. Estimation of the Induction Motor Stator and Rotor Resistance Using Active and Reactive Power Based Model Reference Adaptive System Estimator \- MDPI, 6月 24, 2025にアクセス、 [https://www.mdpi.com/2076-3417/9/23/5145](https://www.mdpi.com/2076-3417/9/23/5145)  
25. A Stator Resistance Estimation Scheme for Speed Sensorless Rotor Flux Oriented Induction Motor Drives \- Slobodan N. Vukosavic, 6月 24, 2025にアクセス、 [http://vukosavic.etf.rs/astatec.pdf](http://vukosavic.etf.rs/astatec.pdf)  
26. Simultaneous Estimation of the Stator and Rotor Resistances in an Induction Motor Drive using Novel Active and Reactive Power Based Model Reference Adaptive System | Request PDF \- ResearchGate, 6月 24, 2025にアクセス、 [https://www.researchgate.net/publication/336874816\_Simultaneous\_Estimation\_of\_the\_Stator\_and\_Rotor\_Resistances\_in\_an\_Induction\_Motor\_Drive\_using\_Novel\_Active\_and\_Reactive\_Power\_Based\_Model\_Reference\_Adaptive\_System](https://www.researchgate.net/publication/336874816_Simultaneous_Estimation_of_the_Stator_and_Rotor_Resistances_in_an_Induction_Motor_Drive_using_Novel_Active_and_Reactive_Power_Based_Model_Reference_Adaptive_System)