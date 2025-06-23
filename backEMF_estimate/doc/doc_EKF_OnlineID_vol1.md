

# **高度なカルマンフィルタ技術を用いたIPMモータのオンラインパラメータ同定：包括的レビュー**

## **1\. 高性能IPMモータ駆動におけるオンラインパラメータ同定の必要性**

### **1.1. 現代の制御戦略における正確なパラメータの役割**

埋込磁石同期モータ（IPMモータ）の高性能制御は、電気自動車（EV）や産業用ロボットなどの先進的な応用分野において不可欠であり、その実現は正確な数学的モデルに大きく依存している 1。特に、ベクトル制御（FOC）や最大トルク/電流制御（MTPA）といった主要な制御戦略は、固定子抵抗（

Rs​）、d-q軸インダクタンス（Ld​, Lq​）、そして永久磁石鎖交磁束（ψf​）といったモータパラメータの正確な値に決定的に依存する 3。これらのパラメータが不正確であると、トルク生成が最適化されず、効率の低下や動的応答性能の悪化を招くことになる。

### **1.2. パラメータ変動という課題**

この分野における中心的な課題は、モータパラメータが一定ではなく、運転中に大きく変動するという点にある。この変動は主に二つの物理現象に起因する。

第一に、**熱的影響**である。固定子抵抗（Rs​）は巻線温度の上昇に伴い増加し、永久磁石鎖交磁束（ψf​）は磁石温度の上昇に伴い減少する。これは、周囲温度が広範囲に変動し、内部での発熱も大きい車載応用などでは特に深刻な問題となる 1。これらの熱的影響を無視すると、性能が低下するだけでなく、磁石の不可逆的な減磁リスクさえ生じさせる 9。

第二に、**磁気飽和**である。d-q軸インダクタンス（Ld​, Lq​）は、鉄心の非線形な磁気特性により、モータの動作点、特にd-q軸電流に応じて変動する。この現象はIPMモータにおいて特に顕著である 6。さらに、一方の軸の磁束が他方の軸の電流に影響される相互インダクタンス（クロスサチュレーション）効果が、モデルを一層複雑化させる 13。

これらの課題に対し、オフラインでの測定や固定されたルックアップテーブルでは予測不可能な運転条件に適応できないため、時々刻々と変化するパラメータをリアルタイムで追跡する**オンライン同定**が唯一の有効な解決策となる 1。このアプローチは、単に制御を最適化するだけでなく、より深い意味を持つ。例えば、温度に依存する

Rs​や$\\psi\_f$を正確にオンラインで同定する技術は、物理的なセンサなしにモータの熱状態を監視する診断ツールとしても機能する。これにより、制御の最適化とシステムの健全性監視という二つの目的が、一つの技術によって同時に達成されるという、相乗的な関係が生まれる 8。

### **1.3. オンライン同定手法の概観**

カルマンフィルタをより広い文脈に位置づけるため、一般的なオンラインパラメータ同定手法を概観する。これには、再帰的最小二乗法（RLS）、モデル規範形適応システム（MRAS）、人工ニューラルネットワーク（ANN）などが含まれる 4。これらの手法の中で、カルマンフィルタは、ノイズの多いセンサ測定値を含むシステムに対して特に適した、モデルベースの確率的推定技術として際立っており、モータ制御応用において広範な研究の対象となっている 4。

## **2\. 動的状態推定のためのカルマンフィルタの理論的基礎**

### **2.1. 状態空間モデル**

カルマンフィルタが用いる中心的な抽象化は、線形の離散時間状態空間モデルである。これは、システムの経時的な振る舞いを記述するための数学的枠組みを提供する 22。このモデルは、二つの基本方程式から構成される。

1. 状態遷移行列（システムモデル）:  
   $$ \\boldsymbol{x}k \= \\boldsymbol{A}\\boldsymbol{x}{k-1} \+ \\boldsymbol{B}\\boldsymbol{u}{k-1} \+ \\boldsymbol{w}{k-1} $$  
   この方程式は、システムの内部状態$\\boldsymbol{x}が、システム行列\\boldsymbol{A}、制御入力\\boldsymbol{u}、そしてプロセスノイズ\\boldsymbol{w}に基づいて、ある時刻ステップから次のステップへとどのように遷移するかを記述する\[27,24\]。プロセスノイズ\\boldsymbol{w}$は、モデル化されていない影響や外乱など、システムダイナミクス自体の不確かさをモデル化する上で極めて重要である 28。  
2. 観測方程式:  
   zk​=Hxk​+vk​

   この方程式は、観測不可能な内部状態$\\boldsymbol{x}を、観測行列\\boldsymbol{H}を介して実際のセンサ測定値\\boldsymbol{z}に関連付ける。この関係は測定ノイズ\\boldsymbol{v}によって乱される\[27,22,24,29\]。測定ノイズ\\boldsymbol{v}$は、センサの不正確さを考慮に入れる 28。

線形カルマンフィルタの最適性の根幹をなすのは、プロセスノイズ$\\boldsymbol{w}と測定ノイズ\\boldsymbol{v}$が、平均ゼロのガウス性白色雑音であるという仮定である 30。

### **2.2. 再帰的な予測・更新サイクル**

カルマンフィルタの核心的な「エンジン」は、各タイムステップで状態推定値を精緻化する、2段階の再帰的なループとして機能する 27。このプロセスは、単なる推定器ではなく、自身の不確かさを動的に管理するメカニズムと解釈できる。誤差共分散行列$\\boldsymbol{P}$は、フィルタが自身の推定値に対して持つ「信頼度」を表し、この不確かさの流れを最適に管理することがフィルタの本質である 22。

* **予測ステップ（時間更新）**:  
  1. 状態予測: x^k−​=Ax^k−1+​+Buk−1​  
     フィルタは、前回の更新された状態推定値とシステムモデルを用いて、現在の状態を予測する 27。上付き文字  
     ⁻は、*事前*（予測）推定値を意味する。  
  2. 誤差共分散予測: Pk−​=APk−1+​AT+Q  
     フィルタは、前回の推定値の不確かさを時間的に前進させ、プロセスノイズの共分散$\\boldsymbol{Q}$を加える。このステップは、不完全なシステムモデルのために時間とともに不確かさが増大する様子を直感的に示す 27。  
* **更新ステップ（観測更新）**:  
  1. カルマンゲインの計算: Kk​=Pk−​HT(HPk−​HT+R)−1  
     カルマンゲイン$\\boldsymbol{K}は、予測に対する信頼度（\\boldsymbol{P}\_k^-）と新たな測定値に対する信頼度（\\boldsymbol{R}$）を最適にバランスさせる重み付け係数として機能する 27。  
  2. 状態更新: x^k+​=x^k−​+Kk​(zk​−Hx^k−​)  
     事前推定値は、観測残差（zk​−Hx^k−​）をカルマンゲインで重み付けして補正される。これが、新たな測定値が予測を精緻化する「フィルタリング」のステップである 27。上付き文字  
     ⁺は、*事後*（更新）推定値を意味する。  
  3. 誤差共分散更新: Pk+​=(I−Kk​H)Pk−​  
     フィルタは、新たな情報に基づいて自身の信頼度を更新する。高いカルマンゲイン（測定値を信頼する）は、共分散（不確かさ）の大きな減少をもたらす 27。

この予測と更新のサイクルは、より広範な再帰的ベイズフィルタリングの具体的な実装と見なすことができる。「予測」ステップは時刻kにおける状態の*事前*確率分布の計算に、「更新」ステップはベイズの定理を用いてこの*事前*確率と新たな観測からの*尤度*を統合し、*事後*確率分布を計算することに対応する 30。線形ガウスシステムでは、これらの分布は常にガウス分布となり、カルマンフィルタの方程式はその平均（状態推定値$\\hat{\\boldsymbol{x}}

）と共分散（行列\\boldsymbol{P}$）を追跡する。

### **2.3. カルマンゲインの最適性**

カルマンゲインの特定の式は、*事後*誤差共分散行列$\\boldsymbol{P}\_k^+のトレース（対角成分の和）を最小化することによって導出される\[27,41,39,40\]。これは、カルマンフィルタが単なるヒューリスティックなフィルタではなく、システムが線形でありノイズがガウス性であるという条件下で、最小平均二乗誤差の意味で∗最適な線形推定器∗であることを意味する\[38,30,32,42\]。導出は、\\boldsymbol{P}\_k^+のトレースを\\boldsymbol{K}\_k$で微分し、その結果をゼロと置くことで行われる 41。

## **3\. IPMモータの非線形パラメータ同定のための拡張カルマンフィルタ（EKF）**

### **3.1. 非線形推定の必要性**

標準的なカルマンフィルタは線形システムに限定される。しかし、IPMモータの電圧方程式は、状態変数の積（例：速度$\\omegaと電流i\_qや磁束\\psi\_d$の積）を含むため、本質的に非線形である 4。したがって、非線形な拡張が必要となる。拡張カルマンフィルタ（EKF）は、この問題に対する最も一般的で歴史的に重要なアプローチであり、各タイムステップで現在の状態推定値の周りでシステムを線形化することによって非線形性に対処する 4。

### **3.2. EKFのためのIPMモータモデルの定式化**

EKFを用いたパラメータ同定の核心的なテクニックは、状態ベクトルの拡張である。未知または時変のパラメータ（Rs​, Ld​, Lq​, ψf​）を、元の状態ベクトル（d-q軸電流id​, iq​）に追加する。これにより、例えば$\\boldsymbol{x} \=^T$のような*拡張状態ベクトル*が生成される 4。

このアプローチの概念的な飛躍は、パラメータをモデル内の固定された定数としてではなく、それ自体が*状態変数*であると見なすことにある 45。パラメータは、非常にゆっくりと変化する状態として、しばしばランダムウォーク（例：

Parameterk​=Parameterk−1​+noise）としてモデル化される 47。このモデル化により、フィルタはパラメータの緩やかな変動を追跡できるようになる。例えば、

Ld​とLq​を同定するためのモデルでは、状態ベクトルを$\\boldsymbol{x} \= \[i\_d, i\_q, 1/L\_d, 1/L\_q\]^T$と定義する 4。

測定可能な出力は通常d-q軸電流であるため、観測方程式はしばしば単純な選択行列となる：y=\[id​,iq​\]T=Hx 4。

重要なのは、状態ベクトルにパラメータを追加することで、本質的な非線形性が導入されることである。IPMモータの電圧方程式にはRs​⋅id​のような項が含まれる。Rs​とid​の両方が拡張ベクトル$\\boldsymbol{x}*{aug}内の状態変数である場合、この項は二つの状態変数の積（例：\\boldsymbol{x}*{aug}(3) \\cdot \\boldsymbol{x}\_{aug}(1)$）となる。この状態変数の乗算は、EKFが線形化によって対処しなければならない強力な非線形性である。これが、拡張状態アプローチ（「Joint EKF」とも呼ばれる）が収束に課題を抱える理由であり、DEKFのような代替アーキテクチャが開発された背景でもある 13。

### **3.3. EKFアルゴリズムの詳細**

EKFの予測・更新サイクルは、線形KFと構造的に似ているが、重要な違いがある。

* 非線形予測: x^k−​=f(x^k−1+​,uk−1​)  
  予測には、完全な非線形状態遷移関数$f(\\cdot)$が使用される。  
* ヤコビ行列による線形化: 線形KFのシステム行列$\\boldsymbol{A}と\\boldsymbol{H}は、非線形関数fとh$を現在の推定値で評価した偏微分であるヤコビ行列に置き換えられる。  
  $$ \\boldsymbol{F}k \= \\frac{\\partial f}{\\partial \\boldsymbol{x}} \\bigg|{\\hat{\\boldsymbol{x}}\_{k-1}^+} \\quad , \\quad \\boldsymbol{H}k \= \\frac{\\partial h}{\\partial \\boldsymbol{x}} \\bigg|{\\hat{\\boldsymbol{x}}\_k^-} $$  
* **EKFの共分散予測と更新**: 予測と更新の各ステップは、線形KFの方程式と同じ構造を持つが、定数の$\\boldsymbol{A}$, $\\boldsymbol{H}$の代わりに、時変のヤコビ行列$\\boldsymbol{F}\_k$, Hk​が使用される 4。  
  * 共分散予測: Pk−​=Fk​Pk−1+​FkT​+Q  
  * 更新ステップ: カルマンゲイン、状態、共分散の更新式は、Hk​を用いて計算される。

### **3.4. EKFの長所と短所**

EKFの主な長所は、より複雑なフィルタと比較して計算効率が高く、実装が比較的容易であるため、多くの実用的な応用で主力となっている点である 4。

一方、その核心的な弱点は線形化ステップにある。システムが非常に非線形である場合、一次のテイラー級数近似は不十分となり、伝播される平均と共分散に大きな誤差を生じさせる可能性がある。これは、性能の低下や、深刻な場合にはフィルタの発散につながる可能性がある 20。また、ヤコビ行列の計算は複雑で、間違いやすい作業となり得る。

## **4\. 精度と収束性を向上させるための高度なカルマンフィルタアーキテクチャ**

EKF、UKF、DEKFは、相互に排他的な代替案ではなく、計算の複雑さ、実装の難易度、および推定精度の間のトレードオフのスペクトル上の点として理解されるべきである。

### **4.1. Unscented Kalman Filter (UKF): 微分不要のアプローチ**

UKFは、EKFの線形化に起因する誤差に直接対処するために導入された 49。その中心的な考え方は、「任意の非線形関数を近似するよりも、確率分布を近似する方が容易である」という点にある 49。

UKFのメカニズムは\*\*Unscented Transform (UT)\*\*に基づいている。関数を線形化する代わりに、UTは状態分布の平均と共分散を捉える決定論的に選択されたサンプル点（「シグマポイント」と呼ばれる）のセットを使用する。これらのシグマポイントは、真の非線形関数を直接通過し、変換された点から新しい平均と共分散が計算される。このアプローチは、事後平均と共分散を少なくとも2次精度で正確に捉えることができるのに対し、EKFは1次精度に過ぎない 49。UKFアルゴリズムはヤコビ行列の計算を必要としないという大きな利点がある 51。

モータ制御におけるUKFとEKFの直接比較に関する文献レビューは、興味深い矛盾を提示している。一部の研究では、特に非線形性の強いシステムや過渡応答時に、UKFがより高い精度を提供するが、計算コストも高いことが示されている 50。しかし、非線形性が顕著な同期リラクタンスモータに関する包括的な実験研究では、EKFが推定誤差の点でUKFを上回り、かつ大幅に高速であるため、その特定の応用においては実用的に優れた選択肢であると結論付けている 50。これは、理論的な優位性が実用的な優位性を保証するものではなく、選択は特定の応用に依存するという重要な示唆を与える。

### **4.2. Dual Extended Kalman Filter (DEKF): 状態とパラメータの分離**

DEKFは、「Joint EKF」（拡張状態）アプローチによって生じる、大規模で強い非線形性を持つシステムという課題への対応策として導入された 47。このアーキテクチャは、「分割統治」戦略を採用し、並列で動作し協調する二つの独立した小規模なEKFを使用する。

* **状態フィルタ**: 主なシステム状態（例：id​,iq​）を推定する。モデル内では、パラメータフィルタからの最新のパラメータ推定値を使用する。  
* **パラメータフィルタ**: モデルパラメータ（例：Rs​,Ld​,Lq​）を推定する。計算には、状態フィルタからの最新の状態推定値を使用する。

この推定問題の「分割」は、二つのより弱い非線形システムをもたらし、収束特性と安定性を向上させると理論付けられている 47。文献は、DEKFが単一の巨大なJoint EKFよりも優れた収束特性を持つことを強く示唆している 13。ある実験結果では、DEKFが200msで収束したのに対し、同等のJoint EKFは4〜5秒を要するという劇的な改善が報告されている 13。また、大規模な行列演算を回避することで、DEKFはJoint EKFよりも計算負荷が低くなる可能性がある 13。

このことから、EKF、UKF、DEKFは、異なる課題に対処するための異なるツールとして理解できる。EKFは基本的なベースライン、UKFは非線形性そのものが主要な課題である場合の高精度な*コンポーネントレベル*の改善、そしてDEKFは合同推定問題の*構造的*な非線形性に取り組むための巧妙な*アーキテクチャレベル*の改善策である。

## **5\. 実世界での実装における重要課題の克服**

### **5.1. 可観測性の問題：ランク落ちと持続的励振**

パラメータ同定の文脈におけるランク落ちの問題は、定常状態（一定の速度と電流）での運転中に、測定された出力信号（電流）が各パラメータ個々の影響を一位に決定するのに十分な動的情報を含まない場合に発生する。システムの可観測性行列がランク落ちし、複数のパラメータの組み合わせが同じ測定値を説明できてしまうため、同時推定が不可能になる 14。文献によれば、定常状態のIPMモータでは、電気的モデルのランクは2であり、同時に同定できるパラメータは2つだけであると報告されている 13。

この問題を克服するための主要な技術は、システムに「持続的励振（persistent excitation）」を導入することである。これは、基本制御信号に高周波信号（通常はd軸またはq軸への正弦波電圧または電流）を重畳して注入することによって達成される 20。この意図的に誘起された動的挙動は、システムが常に十分に「探査」されることを保証し、パラメータのフルセットを可観測にし、同定問題をフルランクにする 57。このことから、ロバストなオンラインパラメータ同定は受動的な観測活動ではなく、能動的なプロセスでなければならないことがわかる。信号注入の必要性は、推定器を単なる「聞き手」から、「観測するために摂動を与える」能動的な参加者へと変貌させる。

ただし、この信号注入は、可聴ノイズ、トルクリップル、追加損失といった望ましくない副作用を引き起こす可能性があるため、注入信号の振幅と周波数の設計には慎重なトレードオフが求められる 20。

### **5.2. 熱的影響と飽和効果の管理**

適切に実装されたオンラインパラメータ同定器は、これらの影響を本質的に補償する。固定子抵抗と磁石磁束が温度によって変化すると、同定器はこれらの変化をリアルタイムで追跡し、更新された値を制御アルゴリズム（例：MTPA計算機）に提供することで、様々な熱条件下で最適な性能を維持する 7。同様に、インダクタンスが負荷電流に応じて変化すると、同定器はそれを追跡し、モータモデルを現在の動作点に効果的に適応させる 4。

さらに、パラメータ同定は診断目的にも利用できる。磁石鎖交磁束（ψf​）と磁石温度の間の既知の関係を確立することにより、ψf​のオンライン推定値を用いて、物理的なセンサなしに永久磁石のリアルタイム温度を推測できる 9。これは、高度な熱保護と健全性監視を可能にする強力な応用である。一部の研究では、熱モデル（集中定数熱回路網など）とカルマンフィルタ（しばしば二重フィルタ構造で）を組み合わせて、直接ロータ温度を推定することが提案されている 17。

### **5.3. ノイズ共分散のチューニングの重要性（QとR）**

実装における極めて重要な実践的側面として、ノイズ共分散行列のチューニングが挙げられる。カルマンフィルタの性能、収束性、安定性は、プロセスノイズ共分散行列$\\boldsymbol{Q}と測定ノイズ共分散行列\\boldsymbol{R}$の正しいチューニングに大きく依存する 4。

* $\\boldsymbol{Q}はシステムモデルへの信頼度を表す。大きな\\boldsymbol{Q}$は、モデルが不正確でパラメータがより速く変化すると予想されることを意味し、フィルタは新しい測定値により速く応答するが、ノイズが多くなる可能性がある。  
* $\\boldsymbol{R}は測定値への信頼度を表す。大きな\\boldsymbol{R}$は、センサがノイズが多いことを意味し、フィルタはモデル予測により強く依存するようになる。

これらの行列のチューニングは、しばしば実験データに基づいて行われる非自明な反復プロセスであり、科学というよりは芸術と見なされることもあるが、その選択はフィルタの最終的な性能に絶大な影響を与える 4。

## **6\. 統合、比較分析、および提言**

### **6.1. 所見の統合**

本レビューを通じて、カルマンフィルタを用いたオンラインパラメータ同定が、IPMモータドライブの性能を最大限に引き出すために不可欠な、成熟かつ効果的な技術分野であることが明らかになった。主要な課題（非線形性、可観測性、パラメータ変動）と、それに対応する解決策（EKF/UKF/DEKFアーキテクチャ、信号注入、適応的チューニング）が体系的に示された。この技術は、単なる制御ループの構成要素ではなく、自己認識能力を持つインテリジェントな駆動システムの中核要素へと進化しつつある。

### **6.2. フィルタアーキテクチャの比較分析**

最終的に、どのフィルタを選択するかは、古典的なエンジニアリングのトレードオフとなる。単一の「最良」のフィルタは存在しない。

* **EKF**は、計算負荷が低く、その有効性が証明されているため、特に非線形性がそれほど深刻でない場合や、計算リソースが非常に限られている場合のベースラインとして推奨される 20。  
* **UKF**は、高い推定精度が最優先され、システムが線形化によって十分に近似できない強い非線形性を示す場合に検討されるべきである。ただし、追加の計算コストが許容されることが条件となる 49。  
* **DEKF**は、*複数のパラメータを同時に同定する*ための高度なアーキテクチャとして強く推奨される。その優れた収束性と管理可能な計算負荷は、不安定または収束が遅い可能性のあるJoint EKFに対する強力な代替案となる 13。

以下の表は、IPMモータのパラメータ同定における各フィルタの特性をまとめたものである。

**表1: IPMモータパラメータ同定のためのカルマンフィルタ派生の比較分析**

| 特徴 | 拡張カルマンフィルタ (EKF) | Unscented Kalman Filter (UKF) | Dual Extended Kalman Filter (DEKF) |
| :---- | :---- | :---- | :---- |
| **中心原理** | ヤコビ行列による線形化 4 | Unscented Transformによる決定論的サンプリング 49 | 状態とパラメータの推定を分離 53 |
| **非線形性の扱い** | 一次近似。強い非線形性には不正確な場合がある 49 | 二次（またはそれ以上）の精度での近似。強い非線形性に対してより正確 51 | 合同推定の構造的非線形性を、より小さなEKF問題に分離することで対処 48 |
| **計算負荷** | 低い。O(n3) 50 | 高い。O(L3)（Lはシグマポイント数）。通常EKFの3-5倍 50 | 中程度。同等のJoint EKFより低く、単純な状態のみのEKFより高い 13 |
| **実装の複雑さ** | 中程度。ヤコビ行列の解析的導出が必要。 | 低い。ヤコビ行列は不要だが、アルゴリズムはより複雑。 | 高い。二つの相互作用するフィルタを実装・調整する必要がある。 |
| **収束特性** | Joint (拡張) EKFアプローチでは、非線形性が強いと収束が遅いか発散する可能性がある 13 | 一般的にロバスト。 | Joint EKFより優れる。実験結果は大幅に速い収束を示す 13 |
| **理想的な使用事例** | 計算コストが主要な制約である場合の汎用的な推定。 | システムの非線形性が支配的な課題である場合の高精度推定。 | 組込みシステム上での、複数のパラメータのロバストで高速な同時推定。 |
| **主要な文献所見** | 実用的なベンチマークとなることが多い 50。一部の実際の駆動実験ではUKFを上回る性能を示すことがある 50。 | シミュレーションと実験の両方で、Joint EKFに対して大幅な性能向上を提供する 13。 |  |

### **6.3. 提言と今後の研究動向**

実務者への提言:  
実践にあたっては、まず適切にチューニングされたEKFから始めることが推奨される。複数のパラメータを同時に同定する場合は、Joint EKFよりもDEKFアーキテクチャの採用を強く推奨する。可観測性の問題を軽視せず、設計の初期段階から信号注入を計画に含めるべきである。  
研究者への提言:  
今後の研究の方向性としては、$\\boldsymbol{Q}および\\boldsymbol{R}$行列の適応的チューニング手法の開発、より高度な非線形フィルタ（例：粒子フィルタ、求積カルマンフィルタ 20）の適用、そしてさらに洗練されたシステムの健全性管理を目指した、推定アルゴリズムと機械学習ベースの熱モデルとのより深い統合などが挙げられる 18。これらの進展は、将来の高性能モータドライブシステムにおいて、自己適応能力と診断能力をさらに向上させる鍵となるだろう。

#### **引用文献**

1. Stator Resistance Estimation Using Adaptive Estimation via a Bank of Kalman Filters \- e-Publications@Marquette, 6月 23, 2025にアクセス、 [https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635\&context=electric\_fac](https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635&context=electric_fac)  
2. Global identification of electrical and mechanical parameters in PMSM drive based on dynamic self-learning PSO, 6月 23, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/126906/1/Global%20Identification%20of%20Electrical%20Parameters](https://eprints.whiterose.ac.uk/id/eprint/126906/1/Global%20Identification%20of%20Electrical%20Parameters)  
3. Online Parameter Estimation for Permanent Magnet Synchronous Machines: An Overview, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/350865564\_Online\_Parameter\_Estimation\_for\_Permanent\_Magnet\_Synchronous\_Machines\_An\_Overview](https://www.researchgate.net/publication/350865564_Online_Parameter_Estimation_for_Permanent_Magnet_Synchronous_Machines_An_Overview)  
4. On-line Parameter Estimation of Interior Permanent Magnet Synchronous Motor using an Extended Kalman Filter \- Korea Science, 6月 23, 2025にアクセス、 [https://koreascience.kr/article/JAKO201407651680963.view?orgId=anpor](https://koreascience.kr/article/JAKO201407651680963.view?orgId=anpor)  
5. On-line Parameter Estimation of Interior Permanent Magnet Synchronous Motor using an Extended Kalman Filter \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/264171595\_On-line\_Parameter\_Estimation\_of\_Interior\_Permanent\_Magnet\_Synchronous\_Motor\_using\_an\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/264171595_On-line_Parameter_Estimation_of_Interior_Permanent_Magnet_Synchronous_Motor_using_an_Extended_Kalman_Filter)  
6. Using on-line parameter estimation to improve efficiency of IPM machine drives, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/3960506\_Using\_on-line\_parameter\_estimation\_to\_improve\_efficiency\_of\_IPM\_machine\_drives](https://www.researchgate.net/publication/3960506_Using_on-line_parameter_estimation_to_improve_efficiency_of_IPM_machine_drives)  
7. Analysis of temperature effects on performance of interior permanent magnet machines | Request PDF \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/313806263\_Analysis\_of\_temperature\_effects\_on\_performance\_of\_interior\_permanent\_magnet\_machines](https://www.researchgate.net/publication/313806263_Analysis_of_temperature_effects_on_performance_of_interior_permanent_magnet_machines)  
8. Parameter identification of PMSM using EKF with temperature variation tracking in automotive applications \- SciSpace, 6月 23, 2025にアクセス、 [https://scispace.com/pdf/parameter-identification-of-pmsm-using-ekf-with-temperature-3lwy82jjzh.pdf](https://scispace.com/pdf/parameter-identification-of-pmsm-using-ekf-with-temperature-3lwy82jjzh.pdf)  
9. Online temperature estimation of IPMSM permanent magnets in hybrid electric vehicles | Request PDF \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/252026119\_Online\_temperature\_estimation\_of\_IPMSM\_permanent\_magnets\_in\_hybrid\_electric\_vehicles](https://www.researchgate.net/publication/252026119_Online_temperature_estimation_of_IPMSM_permanent_magnets_in_hybrid_electric_vehicles)  
10. Online Approach for Parameter Identification and Temperature Estimation in Permanent Magnet Synchronous Machines \- mediaTUM, 6月 23, 2025にアクセス、 [https://mediatum.ub.tum.de/doc/1638373/1638373.pdf](https://mediatum.ub.tum.de/doc/1638373/1638373.pdf)  
11. Inductance Estimation of PMSM Using Extended Kalman Filter, 6月 23, 2025にアクセス、 [https://ijeer.forexjournal.co.in/archive/volume-11/ijeer-110119.html](https://ijeer.forexjournal.co.in/archive/volume-11/ijeer-110119.html)  
12. Comparison of IPMSM Parameter Estimation Methods for Motor Efficiency \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/347045027\_Comparison\_of\_IPMSM\_Parameter\_Estimation\_Methods\_for\_Motor\_Efficiency](https://www.researchgate.net/publication/347045027_Comparison_of_IPMSM_Parameter_Estimation_Methods_for_Motor_Efficiency)  
13. Online Multi-Parameter Observation of IPM Machine ... \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/profile/Zirui-Liu-15/publication/369569211\_Online\_Multi-Parameter\_Observation\_of\_IPM\_Machine\_with\_Reconstructed\_Nonlinear\_Small-Signal\_Model\_Based\_on\_Dual\_EKF/links/6426c6b3315dfb4ccec1091e/Online-Multi-Parameter-Observation-of-IPM-Machine-with-Reconstructed-Nonlinear-Small-Signal-Model-Based-on-Dual-EKF.pdf](https://www.researchgate.net/profile/Zirui-Liu-15/publication/369569211_Online_Multi-Parameter_Observation_of_IPM_Machine_with_Reconstructed_Nonlinear_Small-Signal_Model_Based_on_Dual_EKF/links/6426c6b3315dfb4ccec1091e/Online-Multi-Parameter-Observation-of-IPM-Machine-with-Reconstructed-Nonlinear-Small-Signal-Model-Based-on-Dual-EKF.pdf)  
14. Online parameter estimation for permanent magnet synchronous machines : an overview, 6月 23, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf](https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf)  
15. Parameter Identification and Controller Optimization for Electrical Drives \- mediaTUM \- Technical University of Munich, 6月 23, 2025にアクセス、 [https://mediatum.ub.tum.de/doc/1613501/1613501.pdf](https://mediatum.ub.tum.de/doc/1613501/1613501.pdf)  
16. Abstract of Dissertation, 6月 23, 2025にアクセス、 [https://shizuoka.repo.nii.ac.jp/record/2000126/files/K1275ny.pdf](https://shizuoka.repo.nii.ac.jp/record/2000126/files/K1275ny.pdf)  
17. Real-Time Rotor Temperature Estimation Method for ... \- kth .diva, 6月 23, 2025にアクセス、 [http://kth.diva-portal.org/smash/get/diva2:1737838/FULLTEXT01.pdf](http://kth.diva-portal.org/smash/get/diva2:1737838/FULLTEXT01.pdf)  
18. Parameter Identification of a Permanent Magnet Synchronous Motor Based on the Model Reference Adaptive System with Improved Active Disturbance Rejection Control Adaptive Law \- MDPI, 6月 23, 2025にアクセス、 [https://www.mdpi.com/2076-3417/13/21/12076](https://www.mdpi.com/2076-3417/13/21/12076)  
19. Inductance Estimation of PMSM Using Extended Kalman Filter \- International Journal of Electrical and Electronics Research (IJEER), 6月 23, 2025にアクセス、 [https://ijeer.forexjournal.co.in/papers-pdf/ijeer-110119.pdf](https://ijeer.forexjournal.co.in/papers-pdf/ijeer-110119.pdf)  
20. A Review of Nonlinear Kalman Filter Appling to Sensorless Control for AC Motor Drives, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/338258111\_A\_Review\_of\_Nonlinear\_Kalman\_Filter\_Appling\_to\_Sensorless\_Control\_for\_AC\_Motor\_Drives](https://www.researchgate.net/publication/338258111_A_Review_of_Nonlinear_Kalman_Filter_Appling_to_Sensorless_Control_for_AC_Motor_Drives)  
21. Rotor Position Estimation Approaches for Sensorless Control of Permanent Magnet Traction Motor in Electric Vehicles: A Review \- Semantic Scholar, 6月 23, 2025にアクセス、 [https://pdfs.semanticscholar.org/a751/93a7a8f652651f671bf270cbd79f921ad5ea.pdf](https://pdfs.semanticscholar.org/a751/93a7a8f652651f671bf270cbd79f921ad5ea.pdf)  
22. カルマンフィルタの考え方 \- Logics of Blue, 6月 23, 2025にアクセス、 [https://logics-of-blue.com/kalman-filter-concept/](https://logics-of-blue.com/kalman-filter-concept/)  
23. 状態方程式 \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/stateextrap\_jp.html](https://www.kalmanfilter.net/JP/stateextrap_jp.html)  
24. カルマンフィルタの基礎について詳しく解説！ \- YouTube, 6月 23, 2025にアクセス、 [https://www.youtube.com/watch?v=GH4AficLbLM](https://www.youtube.com/watch?v=GH4AficLbLM)  
25. 素人によるカルマンフィルタの基礎の入門 \#制御工学 \- Qiita, 6月 23, 2025にアクセス、 [https://qiita.com/sakaeda11/items/6b9bfa2e922304b5edab](https://qiita.com/sakaeda11/items/6b9bfa2e922304b5edab)  
26. 状態空間モデルの推論アルゴリズム(カルマンフィルタ・平滑化) \- ぱぐみの部屋, 6月 23, 2025にアクセス、 [https://www.pagumi-bayesian.com/2022/12/31/state-space-model/](https://www.pagumi-bayesian.com/2022/12/31/state-space-model/)  
27. カルマンフィルタの基礎について解説！ \- AGIRobots Blog, 6月 23, 2025にアクセス、 [https://developers.agirobots.com/jp/kalman-filtering/](https://developers.agirobots.com/jp/kalman-filtering/)  
28. カルマンフィルターチュートリアル \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/default\_jp.aspx](https://www.kalmanfilter.net/JP/default_jp.aspx)  
29. 観測方程式 \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/measurement\_jp.html](https://www.kalmanfilter.net/JP/measurement_jp.html)  
30. Kalman Filter入門 \#統計学 \- Qiita, 6月 23, 2025にアクセス、 [https://qiita.com/categoryik/items/3d1ea237998bc1c0da61](https://qiita.com/categoryik/items/3d1ea237998bc1c0da61)  
31. 第1回 カルマンフィルタとは \- 地層科学研究所, 6月 23, 2025にアクセス、 [https://www.geolab.jp/documents/column/kalman01/](https://www.geolab.jp/documents/column/kalman01/)  
32. カルマンフィルタの基礎式を代数とベイズ定理から見る \- am I geek...?, 6月 23, 2025にアクセス、 [http://hamachan-pon.blogspot.com/2017/04/blog-post.html](http://hamachan-pon.blogspot.com/2017/04/blog-post.html)  
33. まとめ \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/multiSummary\_jp.html](https://www.kalmanfilter.net/JP/multiSummary_jp.html)  
34. 第 6 章 多変量カルマンフィルタ \- Python で学ぶベイズフィルタとカルマンフィルタ (翻訳), 6月 23, 2025にアクセス、 [https://inzkyk.xyz/kalman\_filter/multivariate\_kalman\_filters/](https://inzkyk.xyz/kalman_filter/multivariate_kalman_filters/)  
35. シンプルなモデルとイラストでカルマンフィルタを直観的に理解してみる \- Qiita, 6月 23, 2025にアクセス、 [https://qiita.com/MoriKen/items/0c80ef75749977767b43](https://qiita.com/MoriKen/items/0c80ef75749977767b43)  
36. 第 4 章 一次元カルマンフィルタ \- Python で学ぶベイズフィルタとカルマンフィルタ (翻訳), 6月 23, 2025にアクセス、 [https://inzkyk.xyz/kalman\_filter/one\_dimensional\_kalman\_filters/](https://inzkyk.xyz/kalman_filter/one_dimensional_kalman_filters/)  
37. 状態空間モデルとカルマンフィルタの解説 \- GitHub Gist, 6月 23, 2025にアクセス、 [https://gist.github.com/hnakano863/2ba2038c62e0affbb7d07a14a216df4c](https://gist.github.com/hnakano863/2ba2038c62e0affbb7d07a14a216df4c)  
38. カルマンフィルタってなに？ \#機械学習 \- Qiita, 6月 23, 2025にアクセス、 [https://qiita.com/IshitaTakeshi/items/740ac7e9b549eee4cc04](https://qiita.com/IshitaTakeshi/items/740ac7e9b549eee4cc04)  
39. 線形カルマンフィルタってなんでこんな式なの？ \- Qiita, 6月 23, 2025にアクセス、 [https://qiita.com/WandererEng/items/39822f33a10cef8473ee](https://qiita.com/WandererEng/items/39822f33a10cef8473ee)  
40. カルマンフィルタの導出 \- kyo-kutsuzawa, 6月 23, 2025にアクセス、 [https://kyo-kutsuzawa.github.io/technical-note/kalman\_filter.html](https://kyo-kutsuzawa.github.io/technical-note/kalman_filter.html)  
41. カルマンゲイン \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/kalmanGain\_jp.html](https://www.kalmanfilter.net/JP/kalmanGain_jp.html)  
42. Extended Kalman Filter Based Inductance Estimation for Dual Three-Phase Permanent Magnet Synchronous Motors Under the Single Open-Phase Fault | Scilit, 6月 23, 2025にアクセス、 [https://www.scilit.com/publications/b478a8c57295b315a98206aa1ad1e29b](https://www.scilit.com/publications/b478a8c57295b315a98206aa1ad1e29b)  
43. 1次元カルマンゲインの導出 \- Kalman Filter Tutorial, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/JP/KalmanGainDeriv\_jp.html](https://www.kalmanfilter.net/JP/KalmanGainDeriv_jp.html)  
44. The Kalman Gain, 6月 23, 2025にアクセス、 [https://www.kalmanfilter.net/kalmanGain.html](https://www.kalmanfilter.net/kalmanGain.html)  
45. What is the difference between EM and EKF? \- Mathematics Stack Exchange, 6月 23, 2025にアクセス、 [https://math.stackexchange.com/questions/2434775/what-is-the-difference-between-em-and-ekf](https://math.stackexchange.com/questions/2434775/what-is-the-difference-between-em-and-ekf)  
46. Introduction of a Framework for the Integration of a Kinematic Robot Arm Model in an Artificial Neural Network \- Extended Kalman Filter Approach \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/384232808\_Introduction\_of\_a\_Framework\_for\_the\_Integration\_of\_a\_Kinematic\_Robot\_Arm\_Model\_in\_an\_Artificial\_Neural\_Network\_-\_Extended\_Kalman\_Filter\_Approach](https://www.researchgate.net/publication/384232808_Introduction_of_a_Framework_for_the_Integration_of_a_Kinematic_Robot_Arm_Model_in_an_Artificial_Neural_Network_-_Extended_Kalman_Filter_Approach)  
47. Dual and Joint EKF for Simultaneous SOC and SOH Estimation \- CiteSeerX, 6月 23, 2025にアクセス、 [https://citeseerx.ist.psu.edu/document?repid=rep1\&type=pdf\&doi=203634bcae70892a781c567ca83db27f57cad603](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=203634bcae70892a781c567ca83db27f57cad603)  
48. Dual Extended Kalman Filter for the Identification of Time-Varying Human Manual Control Behavior, 6月 23, 2025にアクセス、 [https://ntrs.nasa.gov/api/citations/20170005722/downloads/20170005722.pdf](https://ntrs.nasa.gov/api/citations/20170005722/downloads/20170005722.pdf)  
49. The Unscented Kalman Filter for Nonlinear Estimation \- Harvard University, 6月 23, 2025にアクセス、 [https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf](https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf)  
50. (PDF) Comparison Between UKF and EKF in Sensorless ..., 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/384386178\_Comparison\_between\_UKF\_and\_EKF\_in\_Sensorless\_Synchronous\_Reluctance\_Motor\_Drives](https://www.researchgate.net/publication/384386178_Comparison_between_UKF_and_EKF_in_Sensorless_Synchronous_Reluctance_Motor_Drives)  
51. UKF-Based Parameter Estimation and Identification for ... \- Frontiers, 6月 23, 2025にアクセス、 [https://www.frontiersin.org/journals/energy-research/articles/10.3389/fenrg.2022.855649/full](https://www.frontiersin.org/journals/energy-research/articles/10.3389/fenrg.2022.855649/full)  
52. Unscented Kalman Filter-Based Robust State and Parameter Estimation for Free Radical Polymerization of Styrene with Variable Parameters, 6月 23, 2025にアクセス、 [https://pmc.ncbi.nlm.nih.gov/articles/PMC8912742/](https://pmc.ncbi.nlm.nih.gov/articles/PMC8912742/)  
53. Dual extended Kalman filter for vehicle state and parameter estimation \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/245309351\_Dual\_extended\_Kalman\_filter\_for\_vehicle\_state\_and\_parameter\_estimation](https://www.researchgate.net/publication/245309351_Dual_extended_Kalman_filter_for_vehicle_state_and_parameter_estimation)  
54. Online Multi-Parameter Estimation of IPM Motor Drives with FiniteControl Set Model Predictive Control | Request PDF \- ResearchGate, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/312424355\_Online\_Multi-Parameter\_Estimation\_of\_IPM\_Motor\_Drives\_with\_FiniteControl\_Set\_Model\_Predictive\_Control](https://www.researchgate.net/publication/312424355_Online_Multi-Parameter_Estimation_of_IPM_Motor_Drives_with_FiniteControl_Set_Model_Predictive_Control)  
55. Incremental Parameter Estimation under Rank-Deficient Measurement Conditions \- MDPI, 6月 23, 2025にアクセス、 [https://www.mdpi.com/2227-9717/7/2/75](https://www.mdpi.com/2227-9717/7/2/75)  
56. What is rank deficiency, and how to deal with it? \- Cross Validated \- Stats Stackexchange, 6月 23, 2025にアクセス、 [https://stats.stackexchange.com/questions/35071/what-is-rank-deficiency-and-how-to-deal-with-it](https://stats.stackexchange.com/questions/35071/what-is-rank-deficiency-and-how-to-deal-with-it)  
57. Demagnetization Fault Diagnosis for PMSM Drive System with Dual Extended Kalman Filter, 6月 23, 2025にアクセス、 [https://www.mdpi.com/2032-6653/16/2/112](https://www.mdpi.com/2032-6653/16/2/112)  
58. Robust estimator design for signal injection-based IPM synchronous machine drives, 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/4099520\_Robust\_estimator\_design\_for\_signal\_injection-based\_IPM\_synchronous\_machine\_drives](https://www.researchgate.net/publication/4099520_Robust_estimator_design_for_signal_injection-based_IPM_synchronous_machine_drives)  
59. (PDF) Online Multi-Parameter Observation of IPM Machine with ..., 6月 23, 2025にアクセス、 [https://www.researchgate.net/publication/369569211\_Online\_Multi-Parameter\_Observation\_of\_IPM\_Machine\_with\_Reconstructed\_Nonlinear\_Small-Signal\_Model\_Based\_on\_Dual\_EKF](https://www.researchgate.net/publication/369569211_Online_Multi-Parameter_Observation_of_IPM_Machine_with_Reconstructed_Nonlinear_Small-Signal_Model_Based_on_Dual_EKF)  
60. Sensorless Control Drive of Permanent Magnet Motor Based on a ..., 6月 23, 2025にアクセス、 [https://aast.edu/pheed/staffadminview/pdf\_retreive.php?url=5695\_45\_118\_Sensorless%20CON%20PMSM%20Simple%20on%20line.pdf\&stafftype=staffpdf](https://aast.edu/pheed/staffadminview/pdf_retreive.php?url=5695_45_118_Sensorless+CON+PMSM+Simple+on+line.pdf&stafftype=staffpdf)  
61. Accurate Rotor Temperature Prediction of Permanent Magnet Synchronous Motor in Electric Vehicles Using a Hybrid RIME-XGBoost Model \- MDPI, 6月 23, 2025にアクセス、 [https://www.mdpi.com/2076-3417/15/7/3688](https://www.mdpi.com/2076-3417/15/7/3688)