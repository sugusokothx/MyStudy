

# **IPMモータ用EKFベース磁束・トルク推定器に関する包括的レビューと戦略的提言**

## **1\. 基礎となる線形EKFモデル（フェーズ0）の分析**

提案されている開発ロードマップは、IPM（埋込磁石同期モータ）の高性能トルク制御実現に向けた、体系的かつ論理的なアプローチを示しており、その計画性は高く評価できます。本セクションでは、開発の出発点となるフェーズ0の既存コードと、その基礎となる線形EKF（拡張カルマンフィルタ）モデルについて詳細な分析を行います。これにより、現状のモデルの妥当性を確認しつつ、後続フェーズで取り組むべき課題の根本原因を明確化します。

### **1.1. フェーズ0の状態ベクトルとモデルの前提条件に関する批評**

フェーズ0で実装されているEKFは、状態ベクトルを x=T と定義しています。これは、モータの電気的状態（電流 id​,iq​）と、緩やかに変動する主要パラメータ（固定子抵抗 Rs​、永久磁石鎖交磁束 ψf​）を同時に推定するための、古典的かつ確立された構成です 1。提供されたPythonコードは、このモデルを正確に実装しており、特に

Rs​ と ψf​ の時間変化をゼロ（R˙s​=0, ψ˙​f​=0）と仮定するランダムウォークモデルを採用しています。これは、これらのパラメータが制御周期に比べて非常にゆっくりと変化するという物理的実態を反映した、EKFにおけるパラメータ推定の標準的な手法です 3。

このモデルの根幹をなすのは、以下の線形化された電圧方程式です。

(vd​vq​​)=(Rs​ωe​Ld​​−ωe​Lq​Rs​​)(id​iq​​)+(Ld​0​0Lq​​)(i˙d​i˙q​​)+(0ωe​ψf​​)  
このフェーズ0のモデルにおける決定的な前提条件は、d軸およびq軸インダクタンス（Ld​,Lq​）が定数であるという点です。この仮定は、モータ電流が比較的小さく、鉄心が磁気的に飽和していない領域でのみ妥当性を持ちます 4。しかし、IPMモータが高トルクを出力する際には大きな電流が流れ、鉄心の磁気飽和が顕著になります。磁気飽和が発生すると、

Ld​ と Lq​ は電流の大きさに依存して非線形に変化し、さらにd軸電流がq軸インダクタンスに影響を与える（またその逆も然り）といった「軸間干渉（クロスサチュレーション）」効果も無視できなくなります 4。フェーズ0の線形モデルはこれらの非線形性を完全に無視しているため、高負荷領域でのトルク推定精度が大幅に低下することは避けられません。このモデルの限界こそが、フェーズ1以降で磁束マップを活用した高度な非線形モデルを導入する根本的な動機となります。

### **1.2. 線形モデルにおける Rs​ と ψf​ の可観測性分析**

EKFの設計において、推定対象のパラメータがシステムの入出力から一意に定まるか、すなわち「可観測性（Observability）」を確保することは極めて重要です。フェーズ0のモデルでは、Rs​ と ψf​ の可観測性が常に保証されるわけではありません。

電圧方程式のq軸成分 vq​=Rs​iq​+ωe​Ld​id​+ωe​ψf​+Lq​i˙q​ に着目すると、Rs​ の影響は電圧降下項 Rs​iq​ に、ψf​ の影響は誘起電圧項 ωe​ψf​ に現れます。

1. **低速域における課題**: モータの回転速度 ωe​ が低い場合、誘起電圧項 ωe​ψf​ は非常に小さくなります。この状態では、vq​ の変動の大部分は Rs​iq​ 項に起因するため、EKFは ψf​ の値を正確に観測することが困難になります。フィルタは観測誤差のほとんどを Rs​ の推定値補正に割り当ててしまい、ψf​ の推定精度は低下します。  
2. **軽負荷・高速域における課題**: 逆に、負荷が軽く iq​ が小さい場合、電圧降下項 Rs​iq​ が小さくなり、Rs​ の観測が難しくなります。この場合、フィルタは観測誤差を主に ψf​ の推定値補正に用います。  
3. **パラメータ変動のカップリング**: 実世界のモータでは、温度上昇に伴い固定子抵抗 Rs​ は増加し、永久磁石の磁束 ψf​ は減少します。これらの物理現象は vq​ に対して相反する影響を与える可能性があります。EKFがこれらの影響を分離できない運転領域（例えば、中速・中負荷域）に留まると、一方のパラメータの変動を他方の変動として誤って推定してしまう「パラメータ推定のカップリング」が生じる危険性があります。例えば、温度上昇による真の Rs​ 増加と ψf​ 減少を、フィルタが ψf​ のみの大幅な減少として解釈してしまう可能性があります。これは、磁束推定、ひいては温度推定の精度に致命的な影響を与えます。

提供されたPythonシミュレーションは、一定の高速回転（we \= 200 \* 2 \* np.pi）で動作しており、これは ψf​ の推定には有利な条件ですが、Rs​ の可観測性を検証するには不十分です 2。堅牢なパラメータ推定器を開発するためには、シミュレーションおよび実機テストにおいて、低速・高電流域や動的な速度・負荷変動を含む、より広範な運転プロファイルを適用し、あらゆる条件下での可観測性と推定性能を検証することが不可欠です。

### **1.3. Pythonシミュレーションと組込みCへの実装パスの評価**

提供されているPythonコードは、アルゴリズム開発とオフライン検証のための優れた基盤です。IPMSM\_Simulatorクラスを用いて「真値」を生成し、EKFの推定結果と比較する構成は、開発のベストプラクティスに沿っています。計画されている通り、このフレームワークを実機から記録した電圧・電流波形データの再生テストに活用することは、実装前のアルゴリズムの妥当性を確認する上で極めて有効なステップです。

Pythonで検証済みのアルゴリズムを、numba、cython、あるいはSimulink Coderのようなツールを用いてC/C++コードに変換し、最終的にターゲットMCU/DSPに実装するという開発パスは、現代の組込みシステム開発における標準的なワークフローです。このアプローチにより、開発初期段階ではPythonの持つ高い生産性と可読性を享受し、最終製品ではC/C++の実行性能を確保することができます。このプロセスは、Texas Instruments 6、STMicroelectronics 8、ルネサス エレクトロニクス 10 といった主要な半導体ベンダーが提供する開発エコシステムとも親和性が高いものです。

このフェーズ0の分析から、既存の線形EKFは妥当な出発点であるものの、磁気飽和とパラメータの可観測性という二つの根本的な課題を抱えていることが明らかになりました。これらの課題を克服することこそが、次フェーズ以降の核心的な目標となります。

## **2\. 非線形磁束推定アーキテクチャの比較分析（フェーズ1）**

フェーズ1は、本開発計画の心臓部であり、モータの最も重要な非線形性である磁気飽和にいかにして対処するかを決定する段階です。提案されている二つのアプローチ、フェーズ1-a（磁束の直接推定）とフェーズ1-b（加法的誤差補正）は、根本的に異なる設計思想に基づいています。本セクションでは、両アプローチを理論的メリット、計算コスト、ロバスト性の観点から深く比較分析し、最終的な戦略的推奨を提示します。

### **2.1. 詳細分析：フェーズ1-a（直接磁束状態推定）**

#### **2.1.1. 状態空間定式化と理論的メリット**

状態ベクトルを x=T とするこのアプローチは、概念的に非常に明快です。状態方程式は、鎖交磁束の基本定義式を直接変形して得られます。

ϕ˙​=v−Rs​i−ωe​Jϕ  
ここで、ϕ=\[ϕd​,ϕq​\]T、v=\[vd​,vq​\]T、i=\[id​,iq​\]T、J=(01​−10​) です。

このアプローチの最大の理論的メリットは、その「直接性」にあります。フィルタが内部で推定・管理する状態量が、制御対象である物理量（d-q軸磁束）そのものであるため、モデルの構造が直感的で理解しやすいという利点があります。

#### **2.1.2. 逆写像 i=M−1(ϕ)：リアルタイム性のボトルネックと安定性リスク**

このアプローチの致命的な弱点が、逆写像 i=M−1(ϕ) の計算にあります。状態方程式 ϕ˙​ を計算するためには、現在の電流ベクトル i が必要ですが、フィルタが持つ状態量は推定磁束 ϕ のみです。したがって、EKFの各予測ステップにおいて、ϕ から i を求めるための非線形連立方程式を解かなければなりません。

計画では、この計算に scipy.optimize.root を用いたニュートン・ラフソン法のような反復解法を想定しています。これはオフラインのシミュレーションでは有効ですが、リアルタイム組込みシステムにとっては極めて大きな計算負荷となります 12。反復解法は、収束するまでの計算時間が初期値や磁束マップの形状に依存して変動するため、「最悪実行時間」が保証されません。これは、厳密な周期性が要求されるモータ制御ループにおいて、極めて望ましくない特性です。さらに、初期値の与え方が不適切であったり、磁束マップが複雑な形状（例えば、複数の解を持つ領域）をしていたりする場合、ソルバーが収束しない、あるいは誤った解に収束する可能性も否定できず、EKF全体の安定性を脅かす重大なリスクとなります 14。

#### **2.1.3. 数値ヤコビアン計算がもたらす影響**

状態遷移行列 F（ヤコビアン）を計算するためには、状態方程式の各状態変数に関する偏微分、特に ∂i/∂ϕ が必要になります。電流 i は、逆写像ソルバーという非解析的な関数の出力であるため、この偏微分を解析的に求めることは不可能です。したがって、計画通り数値微分（前方差分など）に頼らざるを得ません。

∂xk​∂fj​​≈ϵfj​(x+ϵek​)−fj​(x)​  
これは、ヤコビアンの各要素を計算するために、負荷の高い逆写像ソルバーを含む状態方程式 f(x) を、状態ベクトルの次元数だけ余分に呼び出す必要があることを意味します。これにより、前項で指摘した計算負荷の問題がさらに深刻化します。

### **2.2. 詳細分析：フェーズ1-b（加法的誤差補正方式 Δϕ）**

#### **2.2.1. モデル誤差補償オブザーバとしての定式化**

状態ベクトルを x=T とするこのアプローチは、より洗練されており、ロバスト性に優れています。この手法は、問題の捉え方を根本的に変えます。すなわち、「真の磁束」全体を推定しようとするのではなく、まず高精度な磁束マップ（LUT）を用いて名目的な磁束 ϕmap(i) を計算し、EKFにはその名目値と真値との「偏差」Δϕ のみを推定させるのです。

ϕtrue​=ϕmap(i)+Δϕ  
これは、プラントモデル（磁束マップ）とフィードバック補正ループ（EKF）を組み合わせる、制御理論における古典的かつ強力な設計アプローチです。EKFの役割は、小さく、かつ緩やかに変動する誤差項を追従することに特化されるため、カルマンフィルタが最も得意とするタスクに集中できます。このアーキテクチャは、参照モデルと調整モデルを比較するMRAS（Model Reference Adaptive System）16 や、電圧モデルと電流モデルを組み合わせて補正を行うアクティブ磁束オブザーバ 17 と思想的に共通しています。

#### **2.2.2. 計算効率とヤコビアン導出**

このアプローチがもたらす計算上の利点は絶大です。モデルは、観測された電流 id​,iq​ を直接使って、名目磁束 ϕmap(i) を参照します。これは、高速かつ実行時間が固定された、単純なLUT補間処理であり、反復解法は一切不要です 19。

電圧方程式は、ϕ=L(i)i+ψf​ の微分形 ϕ˙​=L(i)i˙+(∂L/∂i)ii˙ と、ϕ˙​=v−Rs​i−ωe​Jϕ を組み合わせることで、電流の時間微分 i˙ に関する状態方程式として表現できます。ここに ϕtrue​ を代入すると、

Linc(i)i˙=v−Rs​i−ωe​J(ϕmap(i)+Δϕ)−Δϕ˙​  
ここで Linc=∂ϕ/∂i は増分インダクタンス行列です。この式から、Δϕ が小さいという前提のもと、非線形性は限定的になります。ヤコビアン F の計算には、計画通り ∂ϕmap/∂i が必要となりますが、これは磁束マップからオフラインで事前計算し、別のLUTとして保存しておくことが可能です。これにより、リアルタイムでのヤコビアン計算は、単純なLUT参照処理となり、フェーズ1-aの数値微分とは比較にならないほど高速かつ確定的になります。安川電機が提案する手法では大規模なLUTを不要としていますが 20、これは解析的なインダクタンスモデルを用いていることを示唆します。本計画のように

∂ϕmap/∂i をLUT化するアプローチは、特に飽和が強いモータに対して、より高い精度と一般性を両立させる現実的な選択肢と言えます。

#### **2.2.3. 本質的なロバスト性と電流制御との統合**

このアーキテクチャは、モデル誤差に対して本質的に高いロバスト性を備えています。仮に、ベースとなる磁束マップ ϕmap(i) に多少の誤差があったとしても、システムは破綻しません。EKFが推定する偏差 Δϕ が、その誤差を補償するようにゼロではない定常値に収束するだけです。システムは誤差に適応します。対照的に、フェーズ1-aでは、マップの誤差が逆写像ソルバーの収束失敗を引き起こし、フィルタ全体が発散する直接的な原因となり得ます。

さらに、この構成では電流 id​,iq​ が状態ベクトルに含まれるため、EKFはフィルタリングされた高品位な電流推定値を提供します。この推定電流値を電流制御ループのフィードバックとして利用することで、センサノイズの影響を低減し、電流制御性能そのものを向上させるという副次的なメリットも期待できます。

### **2.3. 戦略的推奨：フェーズ1-bの選択とその正当化**

以上の詳細な比較分析に基づき、リアルタイム組込みシステムへの実装においては、フェーズ1-bが疑いなく優れたアプローチであると結論付けられます。その計算の確実性、モデル誤差に対する優れたロバスト性、そしてモデル誤差補償器としての洗練された構成は、開発リスクを低減し、より高い性能を実現するための最適な選択肢です。フェーズ1-aは、その概念的な単純さとは裏腹に、計算負荷と安定性に関する許容しがたいリスクを内包しています。

以下の比較表は、両アプローチのトレードオフをまとめたものです。

**表1: フェーズ1-aとフェーズ1-bのアーキテクチャ比較**

| 評価項目 | フェーズ1-a (直接磁束状態) | フェーズ1-b (Δφ補正状態) | 根拠 |
| :---- | :---- | :---- | :---- |
| **計算負荷 (1ステップあたり)** | **非常に高い** | **低い** | 1-aは反復的な非線形ソルバーと、ヤコビアン計算のための複数回のモデル評価が必要。1-bはLUT参照と行列演算のみ。 |
| **リアルタイム確定性** | **低い** | **極めて高い** | 1-aの反復ソルバーは実行時間が変動する。1-bの処理はすべて固定時間で完了する。 |
| **モデル誤差へのロバスト性** | **低い** | **高い** | 1-aはマップ誤差がソルバーの失敗や発散に直結しうる。1-bは誤差を状態量Δφが吸収・補償する。 |
| **ヤコビアン実装** | **複雑・低速** | **単純・高速** | 1-aは暗黙的な関数の数値微分が必要。1-bは事前計算されたLUTを用いた解析的導出が可能。 |
| **電流制御との相乗効果** | **間接的** | **直接的** | 1-aでは電流は状態量ではない。1-bはフィルタリングされた高品質な電流推定値を制御ループに提供できる。 |
| **総合評価** | 推奨しない | **強く推奨** | リアルタイム性能、ロバスト性、実装の容易さの全ての面で1-bが優位。 |

この分析に基づき、開発リソースをフェーズ1-bのアーキテクチャに集中させることを強く推奨します。

## **3\. 熱的ダイナミクスの統合によるロバストな補償（フェーズ2）**

モータの性能は温度に大きく依存します。固定子抵抗は温度と共に上昇し、永久磁石の磁力は低下します。これらのパラメータ変化はトルク生成に直接影響するため、高精度な制御を実現するには温度補償が不可欠です 4。フェーズ2では、この温度の影響をいかにしてモデルに組み込むかという、重要な設計判断が求められます。

### **3.1. 温度を「推定状態」とするか「観測入力」とするか**

#### **3.1.1. EKFによる温度推定の分析**

計画では、温度 T を状態ベクトルに加えることが検討されています。これは、以下の一次熱モデルを状態方程式に組み込むことを意味します。

T˙=Cθ​1​(Ploss​−Rθ​T−Tamb​​)  
ここで、Cθ​ は熱容量、Rθ​ は熱抵抗、Tamb​ は周囲温度です。電力損失 Ploss​ は、銅損 (Rs​(id2​+iq2​)) と鉄損から構成され、推定された電流と抵抗 Rs​ の関数となります。

このアプローチは、電気モデルと熱モデルの間に極めて強い「カップリング」を生み出します。これは重大な安定性リスクを伴います。例えば、Rs​ の推定に誤差が生じると、Ploss​ の計算が不正確になり、結果として T の推定値がずれます。そして、その不正確な T の推定値が、今度は Rs​(T) の値を更新するためにフィードバックされ、誤差をさらに増幅させる正のフィードバックループを形成する可能性があります。これにより、フィルタ全体が発散に至る危険性があります。また、熱抵抗 Rθ​ や熱容量 Cθ​ といった熱パラメータを正確に同定することは難しく、これらの値は冷却ファンの動作状況などによっても変動します 23。

#### **3.1.2. 物理センサによる温度観測の分析**

もう一つの選択肢は、固定子巻線やモータケースにサーミスタのような物理的な温度センサを設置することです。この場合、温度 T はもはや推定すべき「状態」ではなく、EKFモデルに対する既知の「観測入力」となります。

このアプローチの最大の利点は、熱の問題と磁気・電気の問題を「デカップリング（分離）」できる点にあります。温度は直接的、高速、かつ高精度に測定されます 24。EKFは、この測定された温度

Tmeas​ を用いて、各ステップで内部パラメータ（Rs​(Tmeas​),ψf​(Tmeas​)）を更新するだけです。これは、はるかに単純で、安定性が高く、ロバストなアーキテクチャです。主な欠点は、物理センサを追加することによるコストと実装の手間ですが、その影響は限定的です。

多くの研究で、センサレスでの温度推定が試みられていますが、その複雑さと精度の限界も指摘されています 26。一方で、サーミスタは非常に高い精度（例：±0.1°C～±0.5°C）と速い応答時間を持ち、特に過渡的な温度変化に対して、モデルベースの推定器よりもはるかに優れた追従性を示します 24。高性能かつ高信頼性が求められるシステムにおいて、直接測定は最も確実なエンジニアリング的選択です。

### **3.2. 温度依存パラメータのモデリング：Rs​(T), ψf​(T), および3次元磁束マップ ϕ(i,T)**

温度の取得方法（推定か測定か）に関わらず、モデルはその影響を正確に反映する必要があります。

1. **Rs​(T)**: 固定子抵抗の温度依存性は、よく知られた線形関係でモデル化できます。α は銅の抵抗温度係数です。Rs​(T)=Rs,ref​(1+α(T−Tref​))  
2. **ψf​(T)**: 永久磁石の鎖交磁束は、温度上昇に伴い減少します。これも一般的に線形関係で近似され、β は磁石材料固有の負の温度係数です 5。ψf​(T)=ψf,ref​(1+β(T−Tref​))  
3. **ϕ(i,T)**: 最も重要な点は、磁気飽和の特性自体が温度に依存するということです。鉄心のB-Hカーブと永久磁石の残留磁束密度は、どちらも温度によって変化します 22。したがって、究極の精度を追求するためには、電流だけでなく温度も考慮に入れた3次元の磁束マップ  
   ϕmap(id​,iq​,T) が必要となります。これは、ϕmap およびその偏微分 ∂ϕmap/∂i のLUTを3次元に拡張することを意味します。これによりLUTのメモリフットプリントは大幅に増加しますが、全運転領域にわたる最高のトルク精度を実現するためには不可欠なステップです。

### **3.3. 戦略的推奨：サーミスタによる直接測定の優先**

高性能、安全クリティカル、あるいは高信頼性が要求されるアプリケーションにおいては、物理センサを用いて熱測定を状態推定から分離することで得られるロバスト性は、サーミスタのわずかな追加コストをはるかに上回る価値を持ちます。結合された熱・電気モデルが持つ発散のリスクは、商用製品においては許容できません。

したがって、固定子温度を測定するためにサーミスタを使用し、温度 T をEKFへの既知の入力として扱うことを強く推奨します。

**表2: 意思決定マトリクス：温度のEKF推定 vs サーミスタ測定**

| 評価項目 | EKFによる推定 | サーミスタによる測定 | 根拠 |
| :---- | :---- | :---- | :---- |
| **精度** | **中～低** | **高い** | 推定精度は損失モデルや熱抵抗の同定精度に依存し、ドリフトしやすい。サーミスタは±0.2°Cといった高精度が実現可能 24。 |
| **過渡応答性** | **遅い** | **速い** | 推定は熱モデルの遅い時定数に支配され、真の温度変化に遅れる。サーミスタは秒単位の応答時間で巻線温度に直接追従する 24。 |
| **システム安定性・ロバスト性** | **低い** | **高い** | 電気系の状態推定と強く結合し、不安定性のリスクを生む。測定は熱の問題を分離し、システム全体のロバスト性を向上させる。 |
| **実装の複雑さ** | **高い** | **低い** | 熱モデル、パラメータ同定(Rθ​,Cθ​)、より複雑なEKFが必要。測定はADCチャネルと単純な回路のみで済む。 |
| **部品コスト (BOM)** | **ゼロ** (ソフトウェアのみ) | **低い** (低コスト部品の追加) | 性能と信頼性の向上というメリットに比べれば、部品コストの増加はごくわずかである。 |
| **総合評価** | リスクが高い | **強く推奨** | 高性能制御において、サーミスタがもたらす安定性、精度、応答性は不可欠である。 |

## **4\. 高度な実装および調整戦略**

本セクションでは、推奨アーキテクチャ（サーミスタ入力を備えたフェーズ1-b）を組込みターゲットに実装する上で直面する、具体的な技術的課題に対する実践的な指針を提供します。

### **4.1. 多次元ルックアップテーブル（LUT）の設計と最適化**

モデル精度の根幹は、そのLUTにあります。推奨するアーキテクチャでは、ϕmap(id​,iq​,T) とその偏微分 ∂ϕmap/∂i(id​,iq​,T) のためのLUTが必要です。

* **データソース**: これらのマップは、有限要素法（FEM）解析 5、または広範囲なダイナモメータ試験によって生成するのが最も信頼性が高い方法です。FEM解析は設計段階で、ダイナモ試験は実機検証段階でそれぞれ重要な役割を果たします。  
* **補間方法**: 計画にある2次元の線形補間（RegularGridInterpolator）は良い出発点です。これを3次元に拡張したトライリニア（三重線形）補間が、3Dマップには適しています。この計算は、最新の32ビットMCUであれば十分にリアルタイムで実行可能です。  
* **グリッド密度**: LUTのメモリサイズと精度は直接的なトレードオフの関係にあります。磁束の変化が非線形になる領域（飽和特性の「膝」の部分など）ではグリッドを密にし、線形的な領域では粗くするといった不均一なグリッド間隔を採用することで、メモリ使用量を抑えつつ精度を維持することが可能です。  
* **メモリ最適化**: ターゲットMCUのRAMやFlashメモリが厳しい制約となる場合、LUTのデータを16ビット固定小数点数で表現する、あるいはデータ圧縮アルゴリズムを導入するなどの最適化が考えられます。

### **4.2. EKF共分散行列（Q, R）の調整に関する実践的指針**

EKF実装における最も挑戦的かつ重要な工程が、プロセスノイズ共分散行列 Q と観測ノイズ共分散行列 R の調整（チューニング）です。計画にあるように対角成分を機械的にスイープする方法も有効ですが、より物理的意味に基づいたアプローチが効率的です。

* **R（観測ノイズ共分散行列）**: R の対角要素は、観測ノイズ、すなわち電流センサのノイズ分散に対応します。この値は、インバータを停止させた状態で電流センサの出力を多数サンプリングし、その統計的分散を計算することで、物理的に妥当な初期値を設定できます。これにより、チューニングの出発点が明確になります。  
* **Q（プロセスノイズ共分散行列）**: Q は、状態方程式（モデル）が現実をどれだけ正確に表現しているかという「信頼度」を表すチューニングパラメータです。  
  * **電流状態 (id​,iq​) のQ**: これらの項は、電圧方程式のモデル化誤差（例えば、インバータのデッドタイムや電圧降下）や離散化誤差を表現します。値を大きくするとモデルへの信頼度が下がり、フィルタは観測値（電流センサの値）により強く追従するようになります。これにより応答は速くなりますが、ノイズに敏感になります。  
  * **磁束偏差状態 (Δϕd​,Δϕq​) のQ**: これらの項は、磁束マップの誤差が時間的にどれだけ変化するかを表します。この誤差は、温度変化などに起因する非常にゆっくりとした変動であると想定されるため、対応するQの要素は極めて小さな値（例：10−8∼10−12）に設定します。これは、Δϕ がほぼ一定であるというモデルへの高い信頼を意味します。  
  * **抵抗状態 (Rs​) のQ**: この項は、温度変化による抵抗値の変化速度を表現します。温度変化も比較的緩やかであるため、中程度の小さな値（例：10−6∼10−8）が適切です。  
* **イノベーションに基づくチューニング**: 完全な適応型EKF 28 の実装は複雑ですが、その中心的な概念である「イノベーション（innovation）」をチューニングの指標として活用できます。イノベーション  
  y=z−Hx^ は、実際の観測値 z とモデルによる予測値 Hx^ との差分です。  
  * チューニング中に、イノベーションが常に大きく、かつノイズが多い場合、それは観測ノイズを過小評価（Rが小さすぎる）しているか、モデルを信頼しすぎ（Qが小さすぎる）ていることを示唆します。  
  * 逆に、過渡応答時に状態推定値が実際の値に追従するのが著しく遅れる場合、それはモデルを信頼しなさすぎ（Qが大きすぎる）ている可能性があります。  
  * このようにイノベーションの振る舞いを監視することで、闇雲なスイープではなく、論理的なチューニングが可能になります。イノベーション共分散は、フィルタ性能を評価する上で重要な指標です 30。

### **4.3. リアルタイム実行：固定小数点 vs. 浮動小数点とコード最適化**

演算方式の選択は、開発効率と実行性能に大きな影響を与えます。

* **浮動小数点演算**: ARM Cortex-M4FやTI C2000シリーズのFPU搭載MCUなど、ハードウェア浮動小数点ユニット（FPU）を持つプロセッサを使用する場合、開発は大幅に簡素化されます。扱える数値のダイナミックレンジが広く、スケーリングを気にする必要がほとんどないため、Pythonのロジックを直接的に移植できます。  
* **固定小数点演算**: FPUを持たないMCUや、最高の電力効率・実行速度が求められる場合に選択されます。全ての変数をオーバーフローしないように、かつ精度を失わないように慎重にスケーリング（Qフォーマット化など）する必要があります。開発工数は大幅に増加しますが、最適化されたコードは非常に高速に動作します。  
* **数値微分における摂動 ϵ の選択**:  
  * **浮動小数点の場合**: ϵ の値は、使用する浮動小数点数の精度に依存します。単精度（float, 32-bit）の場合、マシンイプシロンは 約1.19×10−7 です。倍精度（double, 64-bit）では 約2.22×10−16 です。一般的に、ϵ の良い選択肢は マシンイプシロン​ 付近とされます。計画にある ϵ≈10−6 は単精度では妥当ですが、倍精度では大きすぎて打切り誤差が問題になる可能性があります 31。  
  * **固定小数点の場合**: 単一のマシンイプシロンは存在しません。表現可能な最小値は、選択したスケーリングファクタに依存します。ϵ は、量子化ノイズに埋もれないように状態変数の最下位ビット（LSB）の数倍以上の大きさを持ち、かつ良好な微分近似が得られる程度に小さい値でなければならず、チューニングが難しいパラメータの一つとなります 33。  
  * **ただし、本レポートでは解析的ヤコビアンを用いるフェーズ1-bを推奨しているため、この数値微分の問題は回避されます。**  
* **計算コストの最適化**: EKFは、特に行列の乗算と逆行列の計算において計算コストが高くなります 34。フェーズ1-bの選択により、最も重い逆写像の計算は既に排除されています。さらなる最適化として、MCUベンダーが提供する最適化された線形代数ライブラリ（例：TIのDigital Control Library (DCL) 36）の活用や、行列データをメモリ上で連続的に配置してキャッシュ効率を最大化するなどの手法が有効です。

## **5\. 推奨事項の要約と改訂開発ロードマップ**

本レポートにおける詳細な分析と評価に基づき、IPMモータ用磁束・トルク推定システムの開発を成功に導くための最終的な戦略的推奨事項と、改訂された開発ロードマップを以下に提示します。

### **5.1. 主要な戦略的推奨事項の集約**

1. **加法的補正EKF（フェーズ1-b）の採用**: 状態ベクトルを x=T とするアーキテクチャを最優先で採用する。これにより、優れた計算性能、リアルタイム確定性、およびモデル誤差に対する高いロバスト性が確保される。  
2. **物理的な温度センサの活用**: 固定子温度の直接測定のためにサーミスタを実装する。温度 T をEKFモデルへの既知の入力として扱うことで、熱的推定と磁気的推定を分離し、システム全体の安定性を抜本的に向上させる。  
3. **温度依存の3次元磁束マップの開発**: 究極の精度を達成するため、FEM解析または広範なダイナモ試験に基づき、温度依存性を含む3次元の磁束マップ ϕmap(id​,iq​,T) およびその偏微分マップを生成する。  
4. **事前計算されたヤコビアンLUTの利用**: フェーズ1-bアーキテクチャにおいて、ヤコビアン計算に必要な偏微分項 ∂ϕmap/∂i をオフラインで事前計算し、LUTとして格納する。これにより、リアルタイムでのヤコビアン更新が高速かつ確定的になる。  
5. **原則に基づいたチューニング戦略の導入**: 観測ノイズ行列 R は実測されたセンサノイズ分散から初期値を設定する。プロセスノイズ行列 Q は、状態量の物理的な変化速度（パラメータは遅く、電流は速い）を考慮して設定し、イノベーションベクトル y を診断ツールとして活用し、論理的にチューニングを進める。

### **5.2. 提案する最終開発ロードマップ**

上記の推奨事項を反映し、当初の計画をより具体的かつリスクを低減した形に改訂した開発ロードマップを以下に示します。

**表3: 改訂版 EKF設計・開発ロードマップ**

| フェーズ | 目標 | 状態ベクトル | モデル／ヤコビアンのポイント | 実装タスク |
| :---- | :---- | :---- | :---- | :---- |
| **0\. ベースラインの再確認** | 既存の線形EKFの動作を再確認し、テスト基盤を拡充する。 | x= | Ld​,Lq​ は定数。 | 1\. 既存コードの安定動作を再確認。 2\. \*\*【新規】\*\*動的な負荷・速度変動を含むテストプロファイルを追加し、可観測性の問題をオフラインで確認する。 |
| **1\. コア非線形推定器の実装 (1-bベース)** | ロバストな$\\Delta\\phi$補正方式のEKFを実装し、磁気飽和に対応する。 | x= | ϕ=ϕmap(i)+Δϕ。 ヤコビアンは解析的に導出し、∂ϕmap/∂i はLUTを用いる。 | 1\. \*\*【新規】\*\*FEM/ダイナモ試験で初期の2次元磁束マップ ϕmap(id​,iq​) と偏微分マップ ∂ϕmap/∂i を生成。 2\. 2次元線形補間を行うLUTモジュールを実装。 3\. フェーズ1-bの状態方程式と解析的ヤコビアンをEKFに実装。 4\. 記録データを用いてオフラインでQ,Rを調整。 |
| **2\. 温度補償の統合** | 温度影響をモデルに組み込み、全運転領域での精度を確保する。 | 状態ベクトルはフェーズ1と同じ。 | サーミスタ入力 Tmeas​ を使用。 Rs​(Tmeas​) を更新。 3次元LUT ϕmap(id​,iq​,Tmeas​) とその偏微分LUTを使用。 | 1\. \*\*【新規】\*\*要求される温度範囲で3次元の磁束マップと偏微分マップを特性評価。 2\. 3次元（三重線形）補間をLUTモジュールに実装。 3\. サーミスタのADC読み取りを制御ループに統合。 4\. EKFが Tmeas​ を用いてLUTの参照と$R\_s(T)$の計算を行うように更新。 5\. 熱サイクル試験を含む全システム検証を実施。 |
| **3\. 組込み実装と最終調整** | 完成したEKFをターゲットMCU/DSPに実装し、最適化と最終検証を行う。 | \- | \- | 1\. CコードをターゲットMCUに移植。 2\. メモリ使用量（LUTデータ型など）と計算速度（固定小数点化、最適化ライブラリ活用など）を最適化。 3\. 実機上でQ,Rの最終オンラインチューニングを実施。 4\. ダイナモメータを用いて、全速度・トルク・温度領域でトルク精度を最終評価。 |

このロードマップに従うことで、計算効率、ロバスト性、および精度の面で最適化された、高品質なIPMモータ制御システムの開発が体系的に推進されるものと期待されます。

#### **引用文献**

1. On-line Parameter Estimation of Interior Permanent Magnet ..., 6月 18, 2025にアクセス、 [https://koreascience.kr/article/JAKO201407651680963.view?orgId=anpor](https://koreascience.kr/article/JAKO201407651680963.view?orgId=anpor)  
2. On-line Parameter Estimation of Interior Permanent Magnet Synchronous Motor using an Extended Kalman Filter \- ResearchGate, 6月 18, 2025にアクセス、 [https://www.researchgate.net/publication/264171595\_On-line\_Parameter\_Estimation\_of\_Interior\_Permanent\_Magnet\_Synchronous\_Motor\_using\_an\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/264171595_On-line_Parameter_Estimation_of_Interior_Permanent_Magnet_Synchronous_Motor_using_an_Extended_Kalman_Filter)  
3. Online Identification of Permanent Magnet Flux Based on Extended Kalman Filter for IPMSM Drive With Position Sensorless Control, 6月 18, 2025にアクセス、 [https://matlabtools.com/wp-content/uploads/paper2.pdf](https://matlabtools.com/wp-content/uploads/paper2.pdf)  
4. Online parameter estimation for permanent magnet synchronous ..., 6月 18, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf](https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf)  
5. Improved Current and MTPA Control Characteristics Using FEM-Based Inductance Maps for Vector-Controlled IPM Motor \- MDPI, 6月 18, 2025にアクセス、 [https://www.mdpi.com/1996-1073/16/12/4712](https://www.mdpi.com/1996-1073/16/12/4712)  
6. C2000Ware\_MotorControl\_SDK 4.00.00.00 \- Texas Instruments, 6月 18, 2025にアクセス、 [https://software-dl.ti.com/C2000/c2000\_apps\_public\_sw/c2000ware\_sdk/motorcontrolsdk/4\_00\_00\_00/release\_notes.html](https://software-dl.ti.com/C2000/c2000_apps_public_sw/c2000ware_sdk/motorcontrolsdk/4_00_00_00/release_notes.html)  
7. C2000WARE-MOTORCONTROL-SDK Software development kit (SDK) | TI.com, 6月 18, 2025にアクセス、 [https://www.ti.com/tool/C2000WARE-MOTORCONTROL-SDK](https://www.ti.com/tool/C2000WARE-MOTORCONTROL-SDK)  
8. STM32 Ecosystem for Motor Control \- STMicroelectronics, 6月 18, 2025にアクセス、 [https://www.st.com/content/st\_com/en/ecosystems/stm32-motor-control-ecosystem.html](https://www.st.com/content/st_com/en/ecosystems/stm32-motor-control-ecosystem.html)  
9. X-CUBE-MCSDK \- STM32 Motor Control Software Development Kit (MCSDK), 6月 18, 2025にアクセス、 [https://www.st.com/en/embedded-software/x-cube-mcsdk.html](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)  
10. Whole Speed Range Sensorless Motor Solution \- Renesas, 6月 18, 2025にアクセス、 [https://www.renesas.com/en/applications/industrial/motor-drives-robotics/whole-speed-range-sensorless-motor-solution](https://www.renesas.com/en/applications/industrial/motor-drives-robotics/whole-speed-range-sensorless-motor-solution)  
11. Motor and Inverter Control Solutions \- Renesas, 6月 18, 2025にアクセス、 [https://www.renesas.com/en/applications/industrial/motor-drives-robotics/motor-control-solutions](https://www.renesas.com/en/applications/industrial/motor-drives-robotics/motor-control-solutions)  
12. A Review of State-of-the-art Techniques for PMSM Parameter Identification \- Sci-Hub, 6月 18, 2025にアクセス、 [https://sci-hub.se/downloads/2020-03-14/48/10.1007@s42835-020-00398-6.pdf](https://sci-hub.se/downloads/2020-03-14/48/10.1007@s42835-020-00398-6.pdf)  
13. Real-time Benchmarks Showcasing C2000™ Control MCU's Optimized Signal Chain \- Texas Instruments, 6月 18, 2025にアクセス、 [https://www.ti.com/lit/an/spracw5a/spracw5a.pdf](https://www.ti.com/lit/an/spracw5a/spracw5a.pdf)  
14. Motion Kinematics \- Performance Motion Devices, 6月 18, 2025にアクセス、 [https://www.pmdcorp.com/resources/type/articles/resources/get/motion-kinematics-article](https://www.pmdcorp.com/resources/type/articles/resources/get/motion-kinematics-article)  
15. General purpose inverse kinematics using lookup-tables \- ResearchGate, 6月 18, 2025にアクセス、 [https://www.researchgate.net/publication/261497783\_General\_purpose\_inverse\_kinematics\_using\_lookup-tables](https://www.researchgate.net/publication/261497783_General_purpose_inverse_kinematics_using_lookup-tables)  
16. Dissertation Study on Current Control Performance Improvement and Permanent Magnet Temperature Identification of IPM Motor （IP, 6月 18, 2025にアクセス、 [https://shizuoka.repo.nii.ac.jp/record/2000126/files/K1275.pdf](https://shizuoka.repo.nii.ac.jp/record/2000126/files/K1275.pdf)  
17. Sensorless Control of PMaSynRM Based on Hybrid Active Flux Observer \- MDPI, 6月 18, 2025にアクセス、 [https://www.mdpi.com/2079-9292/14/2/259](https://www.mdpi.com/2079-9292/14/2/259)  
18. Sensorless Speed Control of IPMSM Using Sliding Mode Observer Based on Active Flux Concept \- IIETA, 6月 18, 2025にアクセス、 [https://iieta.org/download/file/fid/48843](https://iieta.org/download/file/fid/48843)  
19. Advanced Torque Control of Interior Permanent Magnet Motors for Electrical Hypercars, 6月 18, 2025にアクセス、 [https://www.mdpi.com/2032-6653/15/2/46](https://www.mdpi.com/2032-6653/15/2/46)  
20. 技術論文 2023 No.1磁気飽和と磁石温度変化を考慮したIPMSMの高精度トルク制御 \- 安川電機, 6月 18, 2025にアクセス、 [https://www.yaskawa.co.jp/technology/technical-paper/detail230823](https://www.yaskawa.co.jp/technology/technical-paper/detail230823)  
21. Identification of Three Phase IPM Machine Parameters Using Torque Tests \- Nottingham Repository, 6月 18, 2025にアクセス、 [https://nottingham-repository.worktribe.com/file/831893/1/Identification%20of%20Three%20Phase%20IPM%20Machine%20Parameters%20Using%20Torque%20Tests.pdf](https://nottingham-repository.worktribe.com/file/831893/1/Identification%20of%20Three%20Phase%20IPM%20Machine%20Parameters%20Using%20Torque%20Tests.pdf)  
22. A Torque Compensation Control Scheme of PMSM considering a Wide Variation of Permanent Magnet Temperature \- International Compumag Society, 6月 18, 2025にアクセス、 [https://www.compumag.org/Proceedings/2017\_Daejeon/papers/\[PD-M2-2\]\_252.pdf](https://www.compumag.org/Proceedings/2017_Daejeon/papers/[PD-M2-2]_252.pdf)  
23. The Use of EKF to Estimate the Transient Thermal Behaviour of Induction Motor Drive, 6月 18, 2025にアクセス、 [https://www.researchgate.net/publication/260190973\_The\_Use\_of\_EKF\_to\_Estimate\_the\_Transient\_Thermal\_Behaviour\_of\_Induction\_Motor\_Drive](https://www.researchgate.net/publication/260190973_The_Use_of_EKF_to_Estimate_the_Transient_Thermal_Behaviour_of_Induction_Motor_Drive)  
24. Temperature Sensors. Thermistors vs Thermocouples \- Ametherm, 6月 18, 2025にアクセス、 [https://www.ametherm.com/blog/thermistors/temperature-sensors-thermistors-vs-thermocouples](https://www.ametherm.com/blog/thermistors/temperature-sensors-thermistors-vs-thermocouples)  
25. Choosing the Right Temperature Sensor: Thermistor vs Thermocouple vs RTD \- SenTec, 6月 18, 2025にアクセス、 [https://cdsentec.com/thermistor-vs-thermocouple-vs-rtd/](https://cdsentec.com/thermistor-vs-thermocouple-vs-rtd/)  
26. US8487575B2 \- Electric motor stator winding temperature estimation \- Google Patents, 6月 18, 2025にアクセス、 [https://patents.google.com/patent/US8487575B2/en](https://patents.google.com/patent/US8487575B2/en)  
27. Thermistor vs RTD Temperature Measurement Accuracy \- Application Note \- BAPI, 6月 18, 2025にアクセス、 [https://www.bapihvac.com/application\_note/thermistor-vs-rtd-temperature-measurement-accuracy-application-note/](https://www.bapihvac.com/application_note/thermistor-vs-rtd-temperature-measurement-accuracy-application-note/)  
28. Sensorless control of marine permanent magnet synchronous propulsion motor based on adaptive extended Kalman filter \- Frontiers, 6月 18, 2025にアクセス、 [https://www.frontiersin.org/journals/energy-research/articles/10.3389/fenrg.2022.1037595/full](https://www.frontiersin.org/journals/energy-research/articles/10.3389/fenrg.2022.1037595/full)  
29. (PDF) A Sage–Husa Prediction Algorithm-Based Approach for Correcting the Hall Sensor Position in DC Brushless Motors \- ResearchGate, 6月 18, 2025にアクセス、 [https://www.researchgate.net/publication/372602134\_A\_Sage-Husa\_Prediction\_Algorithm-Based\_Approach\_for\_Correcting\_the\_Hall\_Sensor\_Position\_in\_DC\_Brushless\_Motors](https://www.researchgate.net/publication/372602134_A_Sage-Husa_Prediction_Algorithm-Based_Approach_for_Correcting_the_Hall_Sensor_Position_in_DC_Brushless_Motors)  
30. Two novel metrics for determining the tuning parameters of the Kalman Filter \- arXiv, 6月 18, 2025にアクセス、 [https://arxiv.org/pdf/1110.3895](https://arxiv.org/pdf/1110.3895)  
31. Comparing Floating Point Numbers, 2012 Edition | Random ASCII \- WordPress.com, 6月 18, 2025にアクセス、 [https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/](https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/)  
32. Floating-Point Comparison \- ACCU, 6月 18, 2025にアクセス、 [https://www.accu.org/journals/overload/31/173/floyd/](https://www.accu.org/journals/overload/31/173/floyd/)  
33. Fixed point vs Floating point number \- Stack Overflow, 6月 18, 2025にアクセス、 [https://stackoverflow.com/questions/7524838/fixed-point-vs-floating-point-number](https://stackoverflow.com/questions/7524838/fixed-point-vs-floating-point-number)  
34. Comparison of full-model and reduced-model EKF based position and speed estimators for sensorless DTC of permanent magnet synchronous machines | Request PDF \- ResearchGate, 6月 18, 2025にアクセス、 [https://www.researchgate.net/publication/282686749\_Comparison\_of\_full-model\_and\_reduced-model\_EKF\_based\_position\_and\_speed\_estimators\_for\_sensorless\_DTC\_of\_permanent\_magnet\_synchronous\_machines](https://www.researchgate.net/publication/282686749_Comparison_of_full-model_and_reduced-model_EKF_based_position_and_speed_estimators_for_sensorless_DTC_of_permanent_magnet_synchronous_machines)  
35. Implementation of Extended Kalman Filter with Optimized Execution Time for Sensorless Control of a PMSM Using ARM Cortex-M3 Microcontroller \- MDPI, 6月 18, 2025にアクセス、 [https://www.mdpi.com/1996-1073/14/12/3491](https://www.mdpi.com/1996-1073/14/12/3491)  
36. C2000™ real-time control MCUs: Digital Control Library \- Introduction | Video | TI.com, 6月 18, 2025にアクセス、 [https://www.ti.com/video/6138407639001](https://www.ti.com/video/6138407639001)  
37. C2000-DIGITAL-CONTROL-LIBRARY Driver or library | TI.com \- Texas Instruments, 6月 18, 2025にアクセス、 [https://www.ti.com/tool/C2000-DIGITAL-CONTROL-LIBRARY](https://www.ti.com/tool/C2000-DIGITAL-CONTROL-LIBRARY)