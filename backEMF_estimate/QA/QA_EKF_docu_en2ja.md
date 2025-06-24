はい、承知いたしました。提供された文献を、要約せずに内容をそのまま日本語に翻訳します。数式も記載し、図は省略しますが、文章中の図番号は原文と紐づくように英語のまま記載します。

-----

### **ページ 1**

Periodicals of Engineering and Natural Sciences
Vol.6, No.2, 2018年11月, pp. 109\~119
オンラインで入手可能: [http://pen.ius.edu.ba](http://pen.ius.edu.ba)

ISSN 2303-4521

**自動車アプリケーションにおける温度変化追跡を伴うEKFを使用したPMSMのパラメータ同定**

Rachid KERID¹, Hicham Bourouina², Réda Yahiaoui³
¹ アルジェリア、ブリダ、Saad Dahlab大学 電気電子工学科
² FUNDAPL研究所、理学部、ブリダ大学、BP. 270 09000、アルジェリア
³ Femto-ST研究所 UMR 6174、ブルゴーニュ＝フランシュ＝コンテ大学、ブザンソン、フランス

E-Mail: r.kerid@ensh.dz, E-Mail: hi.bourouina@gmail.com

**論文情報**

論文履歴:

  * 受信日 2018年5月30日
  * 改訂日 2018年10月21日
  * 受理日 2018年10月26日

**キーワード:**

  * 永久磁石同期機 (PMSM)
  * モデリング
  * 同定
  * 拡張カルマンフィルタ (EKF)
  * トルク制御
  * 温度変化

**要旨**

永久磁石同期機は、その高い電力密度と広い磁束弱め範囲での効率性から、電気自動車の駆動に広く使用されています。本稿では特にEKFを用いたPMSMパラメータの推定に焦点を当て、温度変化がPMSMモータの挙動に与える影響を評価する研究を提示し、それゆえに温度依存パラメータを推定することを提案します。この研究における主な貢献は、パラメータまたはその温度変化を推定するための効果的な手法であり、トルクオブザーバ内のパラメータを追跡・適応させることにより性能劣化を研究し回避することを可能にし、それによって任意の温度で同じ性能を見出すことができます。また、これは熱監視にも使用でき、モータの可用性を向上させ、損傷を引き起こすことなく利用できます。さらに、劣化メカニズムの知識は、この機械の設計に対する洞察も与えます。現在、自動車メーカーが使用しているのは、本質的にトルクと速度に応じた参照電流のマップであり、パラメータ変動は考慮されていません。提案された推定手法の有効性は、シミュレーションと実験の両方によって検証されています。

**連絡先著者:**

Rachid KERID

アルジェリア、ブリダ、
Saad Dahlab大学 電気電子工学科

E-mail: r.kerid@ensh.dz

**1. はじめに**

永久磁石(PM)同期モータは、トラクション、自動車、ロボット工学、航空宇宙技術などの産業用駆動アプリケーションにおいて、近年ますます関心を集めています。誘導モータ駆動装置と比較して、PMモータ駆動装置は高効率、高定常状態トルク密度、および優れた制御性を持ち、広範囲の産業用アプリケーションシステムにおいて優れた代替手段となっています。さらに、低コストのパワーエレクトロニクスデバイスの利用可能性とPM特性の向上により、高効率で高性能なアプリケーション向けの電気モータ駆動装置にPMSMが使用されるようになっています [1]-[3]。PMSMのより正確なパラメータ推定を得るために、文献ではニューラルネットワーク、遺伝的アルゴリズム、Luenbergerオブザーバ、EKFなど、パラメータ変動を追跡するための効果的かつ効率的な代替手段である多くの異なる手法が使用されており、これらはPMSMのパラメータと状態のリアルタイム推定にも使用できます [4]-[6]。

DOI: 10.21533/pen.v6i2.168

109

CC BY

-----

### **ページ 2**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

一方で、いくつかの研究では温度がパラメータに与える影響が考慮されていますが、課題はモータが加熱したときに性能を低下させないようにすることであり、温度に依存するモータの物理的パラメータのトルク変動をオンラインで制御に適応させることです。そうするためには、これらの物理的パラメータをオンラインで決定する必要があり、それにはこれらのパラメータを直接推定するか、またはそれらの変動を温度変動と関連付ける必要があります [7]-[13]。この目的のために、本研究は本質的にEKFを用いた電気モータパラメータのオンライン推定に焦点を当て、それが実装されます。これにより、選択された基準を最適化するパラメータの値を決定することが可能になります。したがって、我々は温度依存パラメータ、すなわち固定子巻線抵抗と磁石磁束を推定することを提案します。これらのパラメータまたはその温度変化の知識は、トルク制御を適応させることによって性能劣化を回避することを可能にし、損傷のリスクなしにモータのより良い可用性を可能にする熱監視を可能にします。

本稿の残りの部分は次のように構成されています：

まず、PMSMの動的モデルをセクション2で説明します。次に、EKFアルゴリズムの開発をセクション3で説明します。セクション4では、EKFを用いたPMSMの多パラメータ同定とシミュレーション結果を示します。次に、これらのパラメータまたはその温度変化の知識と、それがPMSMの挙動に与える影響をセクション5で提示します。最後に、提案された推定手法の有効性をシミュレーションと実験の両方で検証します。

**2. PMSMのDQモデル**

永久磁石同期モータの電気回路図をFigure 1に示します。これは単相の回路図で、$R\_{s}$は固定子抵抗、$L\_{s}$は固定子インダクタンスです。ロータの永久磁石はロータ軸と整列して磁束$\\psi\_{f}$を生成し、それによって固定子誘導電圧$U\_{E}$が生じます。これは供給電圧$U\_{s}$とは逆向きです。

(Figure 1. PMSMの電気回路図)

永久磁石同期モータのモデルを得るために、Figure 1の電気回路図から始めます。採用された仮定は [14], [15] の通りです：

  * 供給電圧は正弦波である
  * 固定子巻線のRとLの値はすべての巻線で同じであり、一定である。
  * 空隙における磁気誘導Bの経過は一定である。
  * 磁気回路は線形であり、鉄損は無視される。
  * 誘導電圧は正弦波である。

パーク変換後、PMSMの方程式は次のようになります：

$\\nu\_{dq}=Rs(https://www.google.com/search?q=i\_%7Bdq%7D)+\\frac{d}{dt}(https://www.google.com/search?q=%5Cpsi\_%7Bdq%7D)+p\\Omega[\\begin{matrix}0&-1\\ 1&0\\end{matrix}](https://www.google.com/search?q=%5Cpsi_%7Bdq%7D)$ (1)

ここで、$V\_{dq}$、$https://www.google.com/search?q=%5Cpsi\_%7Bdq%7D$、および$https://www.google.com/search?q=i\_%7Bdq%7D$は、それぞれd-q座標系における固定子電圧、磁束、および電流の成分です。

110

-----

### **ページ 3**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

$v\_{dq}=Rs(https://www.google.com/search?q=i\_%7Bdq%7D)+[\\begin{matrix}L\_{d}&0\\ 0\&L\_{q}\\end{matrix}]\\frac{d}{dt}(https://www.google.com/search?q=i\_%7Bdq%7D)+p\\Omega[\\begin{matrix}0&-L\_{q}\\ L\_{d}&0\\end{matrix}](https://www.google.com/search?q=i_%7Bdq%7D)+p\\Omega[\\begin{matrix}0\\ \\psi\_{f}\\end{matrix}]$ (2)

ここで、$\\psi\_{f}=M\_{o}i\_{f}$は永久磁石磁束です。

非突極MSAPのパラメータ同定のための新しいモデルは、次のように与えられます [19]：

$[\\begin{matrix}v\_{j}\\ v\_{q}\\end{matrix}]=[\\begin{matrix}i\_{j}&(\\frac{di\_{x}}{dt}-p\\Omega i\_{x})\&p\\Omega2&0\\ i\_{q}&(p\\Omega i\_{x},\\frac{di\_{x}}{dt})&0\&p\\Omega[\\begin{matrix}R\_{x}\\ L\\ E\\end{matrix}]$ (3)

**3. 拡張カルマンフィルタアルゴリズム (EKFA)**

EKFアプローチは、非線形システムの動的非線形構造の状態を推定できる、最小二乗法に基づく最適な再帰的推定アルゴリズムです。これは、無相関のガウスノイズおよび測定ノイズ下での非線形確率的構造の状態の条件付き平均と共分散の確率分布を計算するために使用される最適な推定量です。さらに、EKFアルゴリズムは、状態モデルが非線形の場合に状態変数を推定するために適用できます。白色雑音を伴う非線形離散モデルは、次のように与えられます [16]-[18]：

  * **予測ステップ:**

$x(\\frac{k+1}{k})=f(x(\\frac{k}{k}),u(k))$ (4)

このステップは、時刻$k+1$における状態ベクトルの最初の推定を構築するのに役立ち、次にその分散を決定しようとします。

ここで：

$P(\\frac{k+1}{k})=f(k)P(k)F(k)^{T}+Q$ (5)
$F(k)=\\frac{\\partial f(x(k),u(k))}{\\partial x^{T}(k)}$ (6)

  * **補正ステップ:**

誤差の分散を最小化することにより、以下の式が得られます：

  * カルマンフィルタのゲイン：

$K(k+1)=P(\\frac{k+1}{k}).H(k)^{T}(H(k)P(\\frac{k+1}{k})H(k)^{T}+R)^{-1}$ (7)

ここで：

$H(k)=\\frac{\\partial h(x(k))}{\\partial x(k)}$ (8)

  * 誤差共分散行列：

$P((k+1)/(k+1))=P((k+1)/k-K(k+1)H(k)P((k+1)/k)$ (9)

  * 時刻$k+1$における状態ベクトルの推定：
    $\\hat{x}(k+1)/(k+1)=\\hat{x}(k+1/k)+K(k+1)(Y(k+1)-H(\\hat{x}(k+1)/k))$ (10)

111

-----

### **ページ 4**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

ここで：
QとRはシステムの状態から独立しています。

**4. EKFを用いたPMSMのパラメータ同定**

PMSMのパラメータ同定をシステム最適化問題と考えることができます。同定の原理は、理論モデルの出力と実際のシステムの出力との間の誤差を最小化するすべてのパラメータを探索することです。パラメータ同定に使用される最適化アルゴリズムは、正確な結果を得るのに貢献します。ここで、PMSMのモデルは次のような微分方程式の形で表すことができます [6], [20]-[23]：

$\\dot{x}=f(p,x(t),u(t))$ (11)
$y(t)=g(p,x)$,

ここで、$x(t)=(i\_{d},i\_{q})$は状態ベクトル、$V(t)=(V\_{d},V\_{q})$はシステム入力ベクトル、$p=(R\_{s}, L\_{d}, L\_{q}, \\psi\_{f})$は同定するパラメータベクトル、$\\mathcal{Y}(t)$は測定可能な出力ベクトルであり、$f(p, x(t), u(t))$と$g(p,x)$は線形または非線形のシステムです。パラメータ推定の目的は、未知のパラメータベクトルpを可能な限り正確に同定することです。システムの追跡モデルは次のように記述されます：

$\\hat{x}=f(\\hat{p},\\hat{x}(t),u(t))$ (12)
$\\hat{y}(t)=g(\\hat{p},\\hat{x}),$,

EKFパラメータ同定の構成要素とフローチャートをFigure 2に示します。

(Figure 2. EKFパラメータ同定)
(a): EKFパラメータ同定の構成要素
(b): EKFパラメータ同定のフローチャート

112

-----

### **ページ 5**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

**5. 温度依存パラメータがPMSMの挙動に与える影響**

制御において最良のトルクを確保するためには、常にモータパラメータの値を知る必要がありました。これらのうち、銅の抵抗率を介した固定子巻線の抵抗と、温度上昇とともに減磁する永久磁石の磁束の2つは、走行中に変化するモータ温度に依存します。インダクタンスの値は、温度に依存しません。課題は、モータが加熱したときに性能を低下させないように、温度依存のモータの物理パラメータのトルク変動をオンラインで制御に適応させることです。これを行うには、これらの物理パラメータをオンラインで決定する必要があり、それにはこれらのパラメータを直接推定するか、またはそれらの変動を温度変動と関連付ける必要があります [9]-[13]。実際、デフォルトでは、Figure 5に示すように、トルク制御は定格温度（20℃）でのパラメータ値を使用して設定されます。

したがって、最適化問題（Figure 2）は偏りを持ちます。なぜなら、公称温度で最適であった参照電流は、実際の温度ではもはや最適ではなくなるからです。供給されるトルクは要求されるトルクよりも小さくなり、出力電力についても同様です。特に、動作領域の限界では、最大電力と最大トルクはもはやアクセスできなくなります。したがって、任意の動作点で要求される電力とトルクを供給し、特に利用可能な速度、電力、および最大トルクを維持するためには、最適化問題に正しいパラメータ値を供給する必要があります。この問題を解決するために、任意の温度で同じ性能を見出すための適応型トルク制御があります。現在、自動車メーカーが使用しているのは、本質的にトルクと速度に応じた参照電流${I\_{d}}^{*}$と${I\_{q}}^{*}$のマップであり、パラメータの変動は考慮されていません。

**6. 適用結果と考察**

PMSMパラメータ推定に適用されたEKFの性能を実証するために、Matlab/Simulinkを使用してシミュレーションが実施されます。Figure 3に示すように、固定子抵抗、インダクタンス、およびロータ磁束連鎖などのPMSM電気パラメータの同定のために、EKFを使用して適合度関数が最適化されます。公称パラメータと推定パラメータの値をTable 1に示します。Figure 3(a)では、推定値は実際の抵抗(3.478 Ω)に非常に近く、Figure 3(b)では実際のインダクタンス(0.0125 H)に近いです。しかし、ロータ磁束連鎖は、巻線抵抗とインダクタンスが正確に推定された場合にのみ、正確に推定できます。実験結果が示すように、推定された巻線抵抗とインダクタンスは高い精度を持っているため、EKFによって推定されたロータ磁束連鎖は正確(0.015 Wb)であることがFigure 3(c)で示されています。結果として、提案された手法はパラメータの変動を正確に追跡する上で高い性能を持っています。

一方、我々はEKFをモンテカルロシミュレーションベースの非線形フィルタリングアルゴリズムである粒子フィルタリング（PF）と比較しました。実験から、EKFは収束するために初期状態の良好なガウス推定と測定ノイズの評価を必要とし、PFは少数の粒子ではうまく機能しないこと、再帰的フィルタリングアルゴリズムにおけるフィルタリング手法の平均分散と複雑性、そしてPFはEKFよりも実行時間が少し長いが、その分散は文献[22]で示されているように最小であることがわかっています。

実行されたシミュレーションから、パラメータ推定のために提案されたEKFは、モンテカルロアルゴリズムよりも一貫性があり、収束性が高いことがわかります。また、得られた結果は、最小二乗法およびニュートン・ラフソン法とも比較され、より正確な性能を示しました [24]-[27]。さらに、我々の結果は、精度の点で、文献[9]でLiuらが予測したものと著しく類似していることがわかりました。

113

-----

### **ページ 6**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

**Table 1. PMSMの電気パラメータ。**
| パラメータ | EKF推定 | 公称パラメータ |
|---|---|---|
| $R\_{s}(\\Omega)$ | 3.4782 | 3.4 |
| $L=L\_{d}=L\_{q}(H)$ | 0.0125 | 0.0121 |
| $\\psi\_{f}(Wb)$ | 0.0150 | 0.013 |

(Figure 3(a): 固定子抵抗Rsの推定)

(Figure 3(b): インダクタンス $L=L\_{d}=L\_{q}$ の推定)

114

-----

### **ページ 7**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

(Figure 3(c): 磁束$\\psi\_{f}$の推定)

**6.1. 実験**

提案された推定方法の有効性を検証するために、この目的のために実験が実施されます。実験結果は、負荷としてDCモータに機械的に結合された電流制御駆動によって供給されるPMSMで得られ、DSPベースのハードウェアはFigure 4に示されています。PMSMの技術的特性はTable 2に示されています。

実験では、モータは公称速度で回転し、dq軸電流、dq軸電圧、および電気速度を含むデータが保存されます。同定プロセス中、データは同じプラットフォーム上にあるコンピュータに送信され、ベクトルPのパラメータ値を繰り返し決定します。

このアプリケーションでパラメータを同定するために使用される最適化プロセスをFigure 3に示します。同定結果は、Table 1に示された前述のシミュレーション結果と同様です。

(Figure 4. 実験システムの構成)

**Table 2. PMSMの仕様**

| 電気的パラメータ | 機械的パラメータ |
|---|---|
| 定格電力 1.38 kW | 定格速度 = 3000 rpm |
| 定格電圧 = 220 V | $J=0.0011\~Kg.m^{2}$ |
| 公称電流 = 7.1A | $f=5.10^{-5}mN(rd/s)$ |
| $\\psi\_{f}=0.013$ | $P=2$ |

温度依存パラメータの変化に対するオンラインでの非適応制御に起因する性能劣化、および逆に適応した場合の性能向上をFigure 5（シミュレーション結果）に示します。したがって、公称温度では、制御はその役割を果たし、要求されたトルクを供給します。しかし、温度が異なる場合、トルク制御に使用されるモータの物理パラメータの値は、参考文献[9]、[18]に示されているように、実際の値とは異なります。供給されるトルクは、Figure 5(a)に示すように要求されたトルクよりも小さく、Figure 5(b)で言及されているように、温度上昇による性能劣化にもかかわらず、EKFによって得られるPMSMの機械的速度は固定されたままです。低速でのEKFの性能は満足のいくものではなく、遷移時に高速になります。しかし、測定された速度は、参考文献[8]-[11]で言及されているように、EKFの性能が中速および高速動作で満足のいくものであることを示しています。Figure 5(c)は、一定の速度指令、温度上昇、および最大値に対して変化する負荷トルクを持つ機械的特性の下限と上限の両方を示しています。Figure 5(d)は、PMSMが高速で動作すると全体的な効率が悪影響を受け、機械効率の低下にさらに寄与することを示しています。

115

-----

### **ページ 8**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

(Figure 5. PMSMに対する温度の影響のシミュレーション結果)
(a): 電磁トルク, (b): 速度, (c): 機械的特性, (d): 効率対固定子抵抗

**7. 結論**

電気自動車アプリケーションで使用できるPMSMの効率は、いくつかのパラメータによって影響を受けます。本稿では、温度に依存するパラメータ、すなわち巻線抵抗と磁石磁束を推定することを提案しました。これらのパラメータまたはその温度変化を知ることは、トルク制御におけるパラメータを追跡および適応させることによって性能劣化を回避することを可能にし、損傷を引き起こすことなくモータの可用性を向上させるための熱監視を可能にします。また、PMSMの挙動に対する全体的な影響も評価され、提案された方法の有効性を検証するためにシミュレーションと実験の両方の結果が提供されています。他の異なるアルゴリズムとの比較により、EKFがより優れた最適化能力を持ち、固定子抵抗、dq軸インダクタンス、およびロータ磁束連鎖などのPMSM電気パラメータを同時に推定する際に良好な収束性を持つことを示しました。提案された方法は非突極PMSM（$Ld=Lq$）に使用されます。

将来のアプリケーションで実験的分析を実行するために、我々は実験計画法（DOE）技術の利用を試みます。これは、さまざまなタイプのシステム、プロセス、および製品の設計、開発、最適化で使用される統計ツールです。これは、最適化とロバストなパラメータ推定、変数スクリーニング、および伝達関数同定を比較するための設計など、さまざまな状況で採用される基本的な方法と見なすことができます。

116

-----

### **ページ 9**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

**謝辞**

この研究は、FUNDAPL研究所、ブリダ大学（アルジェリア）に所在するアルジェリアPNRの支援を受けました。

**参考文献**

[1] S. Ichikawa, M.Tomita, S. Doki, and S. Okuma. "Sensorless control of permanent-magnet synchronous motors using online parameter identification based on system identification theory," IEEE Transactions on Industrial Electronics, vol. 53, no. 2, pp.363-372 (2006).

[2] D. Xu, J. Liu, S., Zhang, H. Wei, Elimination of Low-speed Vibration in Vector-controlled Permanent Magnet Synchronous Motor by Real-time Adjusted Extended Kalman Filter. Electric Power Components and Systems, 43(20), 2276-2287 (2015).

[3] R. Olfati-Saber." Distributed Kalman filter with embedded consensus filters. 44th IEEE Conference on Decision and Control", 2005 and 2005 European Control Conference (CDC-ECC '05), pages 8179-8184 (Dec. 2005).

[4] O. Gursoy, M. H. Sharif, Parallel Computing for Artificial Neural Network Training. Periodicals of Engineering and Natural Sciences, 6(1), 1-10(2018).

[5] A. A. Hassan, A. M. Kassem, Modeling, Simulation and Performance Improvements of a PMSM Based on Functional Model Predictive Control. Arabian Journal for Science & Engineering (Springer Science & Business Media BV), 38(11) (2013).

[6] R. Arulmozhiyal and K. Baskaran, "Implementation of a Fuzzy PI Controller for Speed Control of Induction Motors Using FPGA," Journal of Power Electronics, vol. 10, pp. 65-71 (2010).

[7] O. Lutfy, Wavelet Neural Network Model Reference Adaptive Control Trained by a Modified Artificial Immune Algorithm to Control Nonlinear Systems. Arabian Journal for Science & Engineering (Springer Science & Business Media BV), 39(6) (2014).

[8] M.Zeraoulia et al. Diallo, "Electric Motor Drive Selection Issues for HEV Propulsion systems: A comparative study," IEEE Trans on Vehicular Technology, Vol.55, No.6, pp1756-1764 (2006).

[9] L. Liu, W. X. Liu, and D. A. Cartes. Permanent magnet synchronous motor parameter identification using particle swarm optimization. International Journal of Computational Intelligence Research, 4(2), 211-218 (2008).

[10] K. Liu, Q. Zhang, J. Chen, Z.Q. Zhu and J. Zhang. Online multiparameter estimation of nonsalient-pole PM synchronous machines with temperature variation tracking. IEEE Transactions on Industrial Electronics, 58(5), 1776-1788 (2011).

[11] R. Delpoux, M. Bodson, and T. Floquet. "Parameter estimation of permanent magnet stepper motors without mechanical sensors," Control Engineering Practice, vol. 26, pp. 178-187, May 2014.

[12] S.-B. Lee, T. G. Habetler, R. G. Harley, and D. J. Gritter. "An evaluation of modelbased stator resistance estimation for induction motor stator winding temperature monitoring," Energy Conversion, IEEE Transactions on, 17(1):7-15 (2002).

[13] Z. YILMAZ, M. OKSAR, F. BASCIFTCI, Multi-Objective Artificial Bee Colony Algorithm to Estimate Transformer Equivalent Circuit Parameters. Periodicals of Engineering and Natural Sciences (PEN), 5(3), (2017).

[14] G.terorde, "Sensorless control of a permanent magnet synchronous motor for PV-powered water pump systems using the extended Kalman filter," Ninth international Conference on Electrical Machine and Drives, conference Publication N0486, IEE.1999.

[15] K. T. Chau, C. C. Chan, and L. Chunhua, "Overview of permanent-magnet brushless drives for electric and hybrid electric vehicles," IEEE Trans. Ind. Electron. vol. 55, no. 6, pp. 2246-2257, Jun. 2008..

[16] K. I. Laskaris and A. G. Kladas, "Internal permanent-magnetmotor design for electric vehicle drive," IEEE Trans. Ind. Electron., vol. 57, no. 1,pp. 138-145, Jan. 2010.

[17] J. O. Estima and A. J. M. Cardoso, "Performance analysis of a PMSM drive for hybrid electric vehicles," in Proc. Int. Conf. Elect. Mach., Sep. 6-8, 2010, pp. 1-6

[18] N. Urasaki, T. Senjyu, and K. Uezato, "A novel calculation method for iron loss resistance suitable in modeling permanent-magnet synchronous motors," IEEE Trans. Energy Convers., vol. 18, no. 1, pp.41-47

117

-----

### **ページ 10**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

[19] N. Henwood, "Online estimation of electric motor parameters for tracking of its components temperature in automotive application," PhD. thesis, NORMAL SCHOOL SUPERIOR PARIS MINE, France, 2014.

[20] F. khatounian, "Contribution to modeling, identification and control of a haptic interface with one degree of freedom driven by a permanent magnets synchronous machine," PhD. thesis, NORMAL SCHOOL SUPERIOR CACHAN, France, 2006.

[21] S. Ma, P. Wu, J. Ji, X. Li, Sensorless control of salient PMSM with adaptive integrator and resistance online identification using strong tracking filter. International Journal of Electronics, 103(2), 217-231 (2016).

[22] S. Kurak, M. Hodzic, Control and Estimation of a Quadcopter Dynamical Model. Periodicals of Engineering and Natural Sciences, 6(1), 63-75, (2018).

[23] X. Deng, J. Lu, R. Yue, and J. Zhang., A strong tracking particle filter for state estimation. In Natural Computation (ICNC), 2011 Seventh International Conference on (Vol. 1, pp. 56-60). ΙΕΕΕ. (2011, July).

[24] Y. Yi, W. X. Zheng, C. Sun, and L. Guo, DOB Fuzzy Controller Design for Non-Gaussian Stochastic Distribution Systems Using Two-Step Fuzzy Identification. IEEE Transactions on Fuzzy Systems, 24(2), 401-418, (2016).

[25] M. Klingajay and N. I. Giannoccaro, Comparison between least square & Newton Raphson for estimation parameters of an autonomous threaded fastenings. In Industrial Technology, 2003 IEEE International Conference on (Vol. 1, pp. 163-168). IEEE (2003, December).

[26] H. O. Ozer, Y. Hacioglu, N. Yagiz, Controlling the Building Model Using High Order Sliding Mode Control Optimized by Multi Objective Genetic Algorithm. Periodicals of Engineering and Natural Sciences (PEN), 5(3), (2017).

[27] B. Durakovic, Design of Experiments Application, Concepts, Examples: State of the Art. Periodicals of Engineering and Natural Sciences (PEN), 5(3) (2017).

**著者略歴**

**Rachid Kerid** は、2003年5月にアルジェの高等工科学校（アルジェリア）で電気工学の修士号を取得しました。彼はアルジェリアのブリダ大学で臨時研究員となり、そこでブリダ大学の電子工学部でスイッチ、センサー、電気機械の開発に取り組みました。彼は水理学の高等学校で助教として勤務しており、研究対象はナノテクノロジーとワイヤレス電力伝送（WPT）を含みます。

**Hicham Bourouina** は、2010年と2015年にブリダ第1大学（アルジェリア）で電子工学の修士号と物理学の博士号を取得しました。彼はブリダ大学およびブーサーダの高等学校で助教として勤務していました。彼の研究対象は、ロボット工学、マイクロマシン、および電気機械です。

118

-----

### **ページ 11**

Kerid et al.

PEN Vol. 6, No. 2, 2018, pp. 109-119

**Réda Yahiaoui** は、1998年にフランス、オルセーのパリ第11大学で電子センサーおよび集積回路マイクロ波および高速電子の修士号を、2002年に同大学で工学科学の博士号を取得しました。彼は2005年まで電子工学の研究エンジニアおよびUAV開発のプロジェクトマネージャーとして勤務しました。2005年9月以来、彼はFEMTO-ST研究所のマイクロナノ科学・システム（MN2S）部門で助教として勤務しています。フランス、ブザンソン。彼の主な関心は、細菌検出のためのBioMEMSデバイス：設計、モデリング、マイクロファブリケーション、および組み込み電子工学です。

119