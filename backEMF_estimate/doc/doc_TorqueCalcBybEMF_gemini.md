

# **IPM同期モータの高度な磁束推定とトルク補償戦略：温度適応制御に関する包括的レビュー**

## **はじめに**

### **背景と動機**

埋込磁石同期モータ（IPMSM）は、その高効率、高出力密度、そして広範な動作範囲により、電気自動車（EV）、ロボット、産業オートメーションといった高性能アプリケーションにおける基幹技術として確固たる地位を築いています 1。これらの応用分野では、精密かつ迅速なトルク応答がシステムの全体性能を決定づけるため、高度なモータ制御技術が不可欠となります。

### **制御における課題**

IPMSMの高性能制御の中核をなすのが、磁束とトルクの制御を分離（デカップリング）することで、直流機のような線形的なトルク制御を可能にするベクトル制御（Field-Oriented Control, FOC）です 4。しかし、FOCの性能は、制御器が内部に持つモータの数学モデルの精度に決定的に依存します 6。この数学モデルを構成するモータパラメータ（固定子抵抗、インダクタンス、永久磁石磁束）が、実際の運転中に変動することが、FOCの性能を低下させる根本的な要因となります。

### **パラメータ変動の問題**

パラメータ変動の主要因は二つ存在します。一つは電流の大きさに依存する磁気飽和であり、もう一つは本レポートの主題である温度変化です 1。特に温度は、モータの運転に伴う内部損失（銅損、鉄損）によって必然的に上昇し、固定子巻線の抵抗値（

Rs）と、最も重要なパラメータである永久磁石（PM）の磁束鎖交数（ψf）に顕著な影響を及ぼします 1。

### **熱的影響がもたらす結果**

これらの温度に起因するパラメータ変動は、制御性能の低下という形で連鎖的な問題を引き起こします。具体的には、トルク指令値と実際の発生トルクとの間に誤差が生じ、モータの運転効率が低下します。さらに、最大トルク/電流制御（MTPA）や弱め磁束（FW）制御といった高効率・広範囲運転を実現するための最適制御軌道からの逸脱を招き、システムのエネルギー効率を悪化させ、場合によっては運転の安定性を損なうことさえあります 1。この問題こそが、本レポートが解決を目指す中心的な課題です。

### **本レポートの目的と構成**

本レポートの目的は、IPMSMにおける温度起因のパラメータ変動をオンラインで推定し、それを補償するための最先端の手法について、網羅的かつ専門的なレビューを提供することです。読者が温度適応型の磁束推定モデルとトルク補償システムを設計する上で、確固たる理論的・実践的基盤を構築することを目指します。レポートは、まず温度がモータパラメータに与える物理的な影響の基礎から説き起こし、次に多様なオンライン推定手法を比較分析し、最後にそれらの推定値をFOC、特にMTPA制御と弱め磁束制御へ応用する枠組みを論じる構成となっています。

---

## **第1章：IPMモータのパラメータと性能における温度依存性**

本章では、IPMSMのトルク制御における温度問題の物理的および数学的基礎を確立します。

### **1.1. 永久磁石における熱的劣化の物理**

高性能IPMSMで広く採用されるネオジム・鉄・ボロン（NdFeB）磁石の特性は、温度に強く依存します 12。温度が上昇すると、磁石のB-H減磁曲線の形状が変化し、特に残留磁束密度（

Br）が可逆的に減少します。このBrの低下が、PM磁束鎖交数ψfの直接的な減少原因となります 14。NdFeB磁石の温度係数は一般的に負であり、例えば-0.1%/Kといった値が示されます 12。

さらに重要なのは、不可逆減磁のリスクです。モータが高温状態で、かつ弱め磁束制御などで大きな減磁電流（負のd軸電流）に晒されると、磁石の動作点がB-H曲線の「膝点（knee point）」を下回り、磁力が恒久的に失われる可能性があります 15。したがって、磁石温度の監視は、モータの性能維持だけでなく、安全性と信頼性の確保の観点からも極めて重要です。

### **1.2. 固定子巻線抵抗への温度影響**

固定子巻線に使用される銅の電気抵抗Rsは、温度に対してほぼ線形の関係で増加します。この関係は、銅の抵抗温度係数によって規定されます 8。これは比較的単純な物理現象ですが、電圧方程式に基づくオブザーバの精度に直接影響するため、無視することはできません。

### **1.3. d-q軸モデルとトルク生成への影響**

回転座標系（d-q軸）におけるIPMSMの電圧・トルク方程式は、モータ制御の基礎となります 1。

{vd​=Rs​id​+dtdψd​​−ωe​ψq​vq​=Rs​iq​+dtdψq​​+ωe​ψd​​ψd​=Ld​id​+ψf​ψq​=Lq​iq​Te​=23​p(ψd​iq​−ψq​id​)=23​p{ψf​iq​+(Ld​−Lq​)id​iq​}  
ここで、v\_d, v\_qはd-q軸電圧、i\_d, i\_qはd-q軸電流、ψ\_d, ψ\_qはd-q軸磁束鎖交数、L\_d, L\_qはd-q軸インダクタンス、ω\_eは電気角速度、pは極対数、T\_eは発生トルクです。

トルク方程式から明らかなように、ψfの減少は、トルクの主要成分であるマグネットトルク（3/2 \* p \* ψf \* iq）を直接的に減少させます 1。これは、同じq軸電流を流しても発生トルクが低下することを意味し、指令通りのトルクが得られなくなる性能劣化につながります 14。

一方、Rsの増加は、電圧方程式における電圧降下項（Rs\*i）を増大させます。これにより、電圧指令値と実際にモータに印加される逆起電力との関係が変化し、電圧方程式に依存するあらゆるオブザーバの推定精度を悪化させます 6。

さらに、これらの影響は独立していません。ψfの変化は、モータコア内の総磁束密度を変化させます。モータコアの透磁率は磁束密度に依存して非線形に変化する（磁気飽和）ため、ψfの変化がd-q軸インダクタンスLd, Lqを変化させるという二次的な影響（クロスサチュレーション）も生じます 22。この因果関係の連鎖は、問題をさらに複雑にします。

1. 初期観測：温度上昇によりψfが減少する 14。  
2. 関連事実：d-q軸の総磁束は、電流による磁束と永久磁石による磁束の和で構成される 1。  
3. 因果関係：PM磁束成分の変化は、コア内の総磁束密度を変化させる。  
4. 確立された原理：コア材料の透磁率（ひいてはインダクタンス）は、磁束密度の非線形関数である（磁気飽和） 8。  
5. 結論：したがって、温度によるψfの変化は、必然的に二次的かつ電流依存的なLd, Lqの変化を引き起こす。これを無視する単純なモデルでは、特に高飽和IPMSMにおいて重大なモデル化誤差が生じる 1。

### **1.4. 最適制御軌道（MTPAおよび弱め磁束）のドリフト**

#### **MTPA軌道のドリフト**

MTPA制御は、与えられたトルクを最小の固定子電流で実現する制御法であり、銅損を最小化し効率を最大化します。この最適な電流ベクトル（id, iqの組み合わせ）は、ψfとインダクタンス差（Lq \- Ld）に依存します 21。温度上昇によって

ψfが減少すると、この最適軌道は変化します。制御器が「低温時」のMTPA指令電流を使い続けると、運転は最適点から外れ、同じトルクを発生させるためにより多くの電流が必要となり、結果として銅損が増加します 1。

#### **弱め磁束制御への影響**

高速域では、モータの逆起電力がインバータの最大出力電圧に近づくため、電圧制約が支配的になります。この制約下で運転速度をさらに上げるためには、負のd軸電流を流してPM磁束を打ち消す「弱め磁束制御」が必要です 26。この高速域での運転限界は、電流制限円と電圧制限楕円によって定義されます 27。電圧制限楕円の位置と大きさは、

ψfと回転速度に依存します 29。温度上昇による

ψfの減少は逆起電力を低下させ、電圧制限楕円を変化させます。これにより、弱め磁束制御の開始点（基底回転数）や最大出力・最高速度が変動し、あらかじめプログラムされた固定パラメータの弱め磁束戦略は最適性を失い、不安定になる可能性すらあります 30。

| パラメータ | 物理的原因 | d-qモデルへの影響 | MTPA制御への影響 | 弱め磁束制御への影響 | 関連資料 |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **固定子抵抗** Rs | 巻線でのI^2R発熱、銅の特性 | Rs\*i電圧降下の増大 | 所定トルクに対する銅損の増加 | 電圧利用率の低下、損失増加 | 8 |
| **PM磁束鎖交数** ψf | NdFeB磁石のB-H曲線シフト | マグネットトルク成分の減少、飽和を介したLd, Lqへの二次的影響 | 最適電流位相角のドリフト、効率低下 | 電圧制限楕円の変化、基底回転数と最大出力エンベロープの変動 | 1 |

---

## **第2章：オンライン温度・パラメータ推定手法**

本章では、温度によって変動するモータパラメータをリアルタイムで把握するための様々な手法をレビューし、比較分析します。これらの手法は、その基本原理から、電気モデルベース、熱モデルベース、そして両者を融合したハイブリッド・データ駆動型に大別されます。

### **2.1. 電気モデルベースオブザーバ（電気信号からの温度推測）**

このアプローチは、電気パラメータの既知の温度依存性を利用します。オンラインで電気パラメータを観測（推定）し、その値から逆算して温度を推測する手法です 12。

#### **磁束オブザーバ**

磁石温度を推定するための最も直接的な方法です。オブザーバを用いて固定子磁束または回転子磁束を推定し、そのd軸成分からPM磁束ψfを分離します。基準温度（例：20°C）における既知のψf値からの偏差を計算し、磁石の温度係数を用いて温度変化を算出します 12。

この手法の実現における最大の課題は、「積分器問題」です。基本的な磁束オブザーバは逆起電力の積分に依存しますが 6、純粋な積分器は測定誤差やインバータの非線形性に起因する微小なDCオフセットに極めて敏感で、時間と共に誤差が蓄積し、最終的にオブザーバが飽和（ドリフト）してしまいます 20。

この問題に対する解決策として、以下のような改良が提案されています。

* **ローパスフィルタ（LPF）:** 純粋な積分器を一次のLPFに置き換えることで、DCドリフトを抑制できます。しかし、LPFは位相遅れとゲイン誤差を導入するため、特に運転周波数がLPFのカットオフ周波数に近づくと、これらの誤差を補償する必要があります 6。  
* **一般化積分器（SOGI/TOGI）:** 二次および三次一般化積分器（SOGI/TOGI）は、周波数ロックループ（FLL）と組み合わせることで、適応型のバンドパスフィルタとして機能します。これにより、逆起電力の基本波成分を選択的に抽出しつつ、DCバイアスや高調波を効果的に除去でき、単純なLPFよりも優れた推定精度を提供します 32。特にTOGIは、追加のパラメータなしでDCバイアスを効果的に抑制できると報告されています 33。

#### **状態・パラメータ推定器**

* **再帰的最小二乗法（RLS）:** モータの電圧方程式に測定データをフィッティングさせ、パラメータを推定するアルゴリズミックな手法です。強力ですが、ノイズに敏感であり、共分散行列の「ワインドアップ」を避けるためには持続的な励起（十分に情報量の多い信号）が必要です 34。RLSの精度向上のための新しいチューニング手法も研究されています 34。  
* **拡張カルマンフィルタ（EKF）:** ノイズの多いシステムに適した確率的なオブザーバです。パラメータ（Rs, ψf）を付加的な状態量として扱うことで、システムの状態（電流）とパラメータを同時に推定します。一般的にRLSよりもノイズに対してロバストですが、計算負荷は高くなります 37。

#### **高度なオブザーバコンセプト**

* **スライディングモードオブザーバ（SMO）:** パラメータの不確かさや外乱に対するロバスト性で知られています。不連続な制御則を用いて推定状態を「スライディング平面」に拘束することで、外乱を効果的に排除します。ロバストな磁束推定の有力な候補ですが、「チャタリング」と呼ばれる高周波振動の問題を伴うことがあります 31。  
* **線形パラメータ変動（LPV）オブザーバ:** 現代制御理論に基づくアプローチで、オブザーバゲインを変動するパラメータ（速度など）に応じてスケジュールします。これにより、広範な運転領域にわたってロバスト性と安定性が確保されます。LPVオブザーバは、Rsの不確かさを考慮しつつ、劣化したPM磁束とトルクを推定できます 41。ゲインはしばしば線形行列不等式（LMI）を解くことで計算されます 41。  
* **回帰モデルベースオブザーバ:** モータ方程式を線形回帰形式（y \= ΦT \* x）に再構成し、標準的な勾配降下法などを用いてパラメータを推定します。モデル自体に回転速度情報を含まない場合が多く、実装が簡素化される利点があります 43。

### **2.2. 熱モデルベース推定（熱の流れからの温度推測）**

このアプローチは、モータを相互に接続されたノードからなる熱システムとしてモデル化します。入力として発熱量（損失）を与えることで、モデルは各ノードの温度を予測します 14。

#### **集中定数熱回路網（LPTN）**

* **構築:** モータを物理的な構成要素（固定子巻線、固定子ヨーク、回転子、PM、ハウジングなど）に対応するノードに離散化し、それらを熱抵抗（熱伝導と熱伝達を表す）と熱容量（蓄熱を表す）で接続します 46。  
* **入力（損失モデルの重要性）:** LPTNの精度は、その入力である電力損失の精度に決定的に依存します。これには銅損（I^2\*Rs）、鉄損（ヒステリシス損と渦電流損、周波数と磁束に依存）、機械損が含まれます 14。特に鉄損の正確なモデル化は困難な課題です 14。  
* **LPTNのパラメータ化とチューニング:** 熱抵抗と熱容量の値を決定することは、形状、材料特性、製造上のばらつき（例：部品間の接触熱抵抗）に依存するため、簡単ではありません 49。パラメータは、解析計算、有限要素法（FEA）、または実験測定から導出されます 48。実用的なアプローチとして、実験データに対してシステム同定を行い、測定温度と一致するように最適化アルゴリズムを用いてLPTNパラメータをチューニングする方法が一般的です 14。この際、感度解析を行い、PM温度など重要な部品の温度に最も影響を与えるパラメータ（例：ライナーと積層鉄心の接触熱抵抗、エアギャップの熱伝達率）を特定し、キャリブレーションの労力を集中させることが重要です 49。

### **2.3. ハイブリッドおよびデータ駆動型技術**

#### **ハイブリッド電気・熱オブザーバ**

この先進的なアプローチは、応答の速い電気オブザーバ（例：磁束オブザーバ）と、応答は遅いが安定しているLPTNを組み合わせ、両者の長所を活かすものです 53。LPTNは安定的で長期的な温度のベースラインを提供し、不正確な損失モデルに起因するドリフトを抑制します。一方、磁束オブザーバは動的な負荷変動に伴う急な温度変化を迅速に捉え、LPTNの遅れを補います 46。これにより、高速な動的応答と長期的な安定性を両立した、ロバストな多重時間スケールの推定システムが構築されます。特に、電気的励起が不十分な領域では熱モデルが推定を主導し、逆に動的な領域では電気モデルが主導するという、真にハイブリッドな戦略が提案されています 54。

#### **信号注入法**

モデルベースのオブザーバは、逆起電力がノイズに埋もれてしまう低速・停止域では機能しません（「可観測性の問題」） 35。この問題を解決するのが高周波（HF）信号注入法です。基本波駆動電圧に加えて、微小なHF電圧信号を重畳します。モータの突極性（

Ld ≠ Lq）がこのHF信号を変調し、結果として生じるHF電流に回転子の位置情報が含まれるようになります 57。この技術は、低速域における温度依存パラメータ（HFインダクタンスなど）の推定にも応用できます 13。ただし、HF信号注入は可聴ノイズ、トルクリップル、追加損失といったトレードオフを伴います 60。近年の研究では、擬似ランダム信号の使用、注入周波数とPWM周波数の同期、d軸とq軸両方への信号注入によるトルクリップル相殺など、ノイズ低減技術が焦点となっています 60。

この可観測性の問題は、制御戦略そのものを規定します。EVのような広範な速度域を要求されるアプリケーションの制御システムは、必然的にハイブリッド構成とならざるを得ません。つまり、低速域では信号注入法を用い、中高速域ではモデルベースオブザーバに切り替える必要があります 43。そして、この二つの手法間をいかに滑らかに移行させるかが、重要な実装上の課題となります 66。

#### **機械学習およびAIベースのアプローチ**

* **ニューラルネットワーク（NN）:** NNは、モータの入力（電流、速度、冷却条件）と温度との間の複雑で非線形な関係を学習することができます 68。これにより、物理的なパラメータ化を必要としない、高精度な「ブラックボックス」熱モデルとして機能します。MLP、LSTM、CNNなど様々なアーキテクチャが検討されています 68。  
* **物理情報ニューラルネットワーク（PINN）:** これは、モータの支配方程式（微分方程式など）を学習時の損失関数に組み込む最先端のアプローチです 70。これにより、NNは物理的に妥当な解を生成するように制約され、純粋なデータ駆動型手法と比較して精度とデータ効率が向上します 72。ロバストなパラメータ推定のための非常に有望な研究分野です。

| 推定手法 | 基本原理 | 代表的な精度 | 動的応答 | 計算負荷 | ノイズ/不確かさへのロバスト性 | 主な適用範囲 | 主な長所/短所 | 関連資料 |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| **磁束オブザーバ (LPF)** | 逆起電力のLPF積分 | 中 | 中 | 低 | 低 | 中・高速 | 長所: 単純。短所: 位相遅れ/ゲイン誤差の補償が必要。 | 6 |
| **磁束オブザーバ (SOGI)** | 適応バンドパスフィルタ | 高 | 速 | 中 | 中 | 中・高速 | 長所: DCオフセット/高調波除去に優れる。短所: FLLの設計が必要。 | 32 |
| **RLS** | 再帰的最小二乗法 | 高（励起依存） | 速 | 中 | 中 | 全範囲（励起要） | 長所: 高速収束。短所: 持続的励起が必要、ノイズに敏感。 | 34 |
| **EKF** | 拡張カルマンフィルタ | 高 | 速 | 高 | 高 | 全範囲 | 長所: ノイズにロバスト。短所: 計算負荷が高い、チューニングが複雑。 | 37 |
| **LPTN** | 熱伝導・伝達モデル | 中（損失モデル依存） | 遅 | 低 | 中 | 全範囲（定常状態） | 長所: 直感的で安定。短所: 損失モデルの精度に依存、動的応答が遅い。 | 14 |
| **信号注入法** | 突極性を利用した変調 | 高 | 速 | 中 | 高 | 低速・停止 | 長所: 低速での可観測性を確保。短所: ノイズ、リップル、追加損失。 | 13 |
| **ハイブリッド電気・熱** | 電気/熱モデルの融合 | 高 | 速 | 中〜高 | 高 | 全範囲 | 長所: 高速応答と長期安定性を両立。短所: 構造が複雑になる。 | 53 |
| **LPVオブザーバ** | ゲインスケジュール制御 | 高 | 速 | 高 | 高 | 全範囲 | 長所: 広範囲で安定性とロバスト性を保証。短所: LMI求解など設計が高度。 | 41 |
| **ニューラルネットワーク** | データ駆動型学習 | 高〜非常に高い | 速 | 高（学習時） | モデル依存 | 全範囲 | 長所: 複雑な非線形性をモデル化可能。短所: 大量の学習データが必要、物理的妥当性の保証がない。 | 68 |

---

## **第3章：FOCにおける温度適応型トルク補償**

本章では、第2章で推定したパラメータを、ユーザの最終目的であるトルク補償に適用する方法について詳述します。

### **3.1. 適応型MTPA制御**

MTPA制御の目的は、電流制限内でトルクを最大化することです。この制御は、モータが電流制限下にある低中速域で主に使用されます。

#### **MTPA方程式と問題の可視化**

最適なMTPA電流を与える電流位相角は、ψfとLd, Lqに依存する解析式で与えられます 21。温度上昇によって

ψfが変化すると、この最適点が移動します。電流・電圧制限円（楕円）を用いた図で視覚化すると、「低温時」のMTPA点が、モータの温度上昇に伴い、もはや最小電流点ではなくなっていることが明確に示されます。固定されたMTPA点で運転を続けることは、不必要な銅損を発生させることになります 1。

#### **補償戦略**

* **ルックアップテーブル（LUT）:** 最も単純なアプローチです。様々なトルク、速度、温度の組み合わせに対して、最適なid, iq電流を事前に計算または実測し、多次元テーブルに格納します 5。  
  **批判的評価:** LUTの作成には膨大な時間と労力が必要であり、製造上の個体差や経年劣化には対応できません 10。  
* **モデルベース再計算:** 第2章の手法でオンライン推定したψf（および必要に応じてLd, Lq）を用いて、MTPA方程式をリアルタイムで繰り返し解き、最適な電流指令値を動的に生成します 1。高い精度と適応性を実現できます。  
* **探索ベース/信号注入法:** これらは「モデルフリー」なアプローチです。微小な摂動信号（実信号または仮想信号）を注入し、トルク対電流位相角曲線の勾配を探索します。制御器がこの勾配をゼロにするように動作することで、正確なパラメータ値を知ることなく、能動的にMTPA点を探索します 7。パラメータ誤差に強い反面、動的応答が遅くなる可能性があります。  
* **トルクフィードバック制御:** 推定したトルク値をフィードバックし、指令値との誤差を用いて電流指令を調整する閉ループ制御です。この手法の精度は、トルク推定器の精度に完全に依存します（そしてトルク推定器自体がψfに依存します） 11。

### **3.2. 適応型弱め磁束（FW）制御**

高速域では、制御目標は電流最小化（MTPA）から、インバータの最大電圧制約下でのトルク最大化（MTPV: Maximum Torque Per Volt、またはMPP: Maximum Power Point）へと移行します 26。

#### **FWにおける温度の影響**

温度上昇によるψfの減少は、逆起電力を低下させます。固定された「低温時」のψf値を用いる制御器は、弱め磁束のために不必要に大きな負のd軸電流を流してしまい、これは最適性を欠き、損失を増大させます 30。

#### **補償戦略**

* **電圧フィードバック制御:** 一般的でロバストな手法です。電圧指令ベクトルの大きさをPI制御器で調整し、その出力でd軸（弱め磁束）電流を制御します。ψfの変動には自動的に適応します。なぜなら、ψfが低下すると自然に電圧指令が下がり、PI制御器が要求する弱め磁束電流も少なくなるためです 76。  
* **モデルベースFW:** オンラインで推定したψfを用いて、所定の速度とq軸電流に対して電圧ベクトルを制限値内に収めるために必要なd軸電流を直接計算します。フィードバックループよりも高速なダイナミクスが期待できます 30。特許文献 30 には、回転子温度に基づいて電圧指令を生成し、その誤差から電流指令を補正するシステムが記載されており、これは温度補償されたFW制御の明確な一例です。  
* **モデル予測制御（MPC）:** MPCはこの問題に自然に適しています。電圧・電流制限を最適化問題の陽な制約条件として組み込みます。推定されたψfで更新されたモータモデルを用いることで、MPCは全ての制約を満たしつつトルクを最大化する最適な電流を毎制御周期で見つけ出すことができます 19。

制御戦略は、速度と共に進化しなければなりません。低速域ではシステムは電流制限下にあり、目標はMTPA（効率）です。高速域では電圧制限下に移り、目標はFW（最大出力）となります。温度補償スキームは、これら両方の領域に適応できなければなりません。適応型MTPA戦略 1 を実装しても、適応型FW戦略 30 がなければ、モータはどちらかの運転領域で性能が低下します。これは、統一された補償アーキテクチャの必要性を示唆しています。推定された温度/磁束は、MTPAロジックとFWロジックの両方に供給され、全速度域にわたる最適性能を確保する必要があります。

| 戦略 | 原理 | ψf推定への依存度 | 実装の複雑さ | 動的性能 | 最適性 | 関連資料 |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| **MTPA-LUT** | オフライン事前計算 | 間接的（テーブル選択） | 低 | 速 | 準最適（固定LUT） | 5 |
| **MTPA-再計算** | オンライン方程式求解 | 直接的（高精度要） | 高 | 速 | 最適（モデルが完璧な場合） | 1 |
| **MTPA-探索** | 勾配探索 | なし（モデルフリー） | 中 | 遅くなる可能性あり | ほぼ最適 | 7 |
| **FW-電圧フィードバック** | 閉ループ電圧調整 | 間接的（誤差に強い） | 中 | 中 | ほぼ最適 | 76 |
| **FW-モデルベース** | オンライン直接計算 | 直接的（高精度要） | 高 | 速 | 最適（モデルが完璧な場合） | 30 |

---

## **第4章：統合と実装のフレームワーク**

本章では、これまでの分析結果を統合し、実用的な実装に向けた指針と将来展望を示します。

### **4.1. システム設計のための意思決定フレームワーク**

最適な手法は一つではなく、アプリケーションの要求仕様に応じたトレードオフによって決定されます。

* **軸1：性能要求:** 最高の精度と効率が求められるアプリケーション（例：プレミアムEV）では、モデルベースの再計算によるMTPA/FW制御と組み合わせたハイブリッド電気・熱オブザーバが有力な選択肢となります 1。コストと簡素性が優先される場合は、単純なLPTNや多温度LUTアプローチも許容されるかもしれません 11。  
* **軸2：計算機予算:** 選択は利用可能なプロセッサの能力に制約されます。EKF 37、LPV 42、MPC 77 は計算負荷が高く、高性能なDSPやFPGAを必要とする場合があります。LPFベースのオブザーバ 6 やLUT 5 のような単純な手法は、低コストのマイクロコントローラに適しています。  
* **軸3：運転範囲:** 停止・低速域でのロバストな性能が必須のアプリケーションでは、信号注入法の採用は避けられません 43。その場合、設計の焦点は滑らかな移行ロジック 64 とノイズ低減策 60 になります。主に中高速域で運転するアプリケーションでは、より単純な逆起電力オブザーバで十分な場合もあります 33。

### **4.2. 重要な実装上の考慮事項**

* **インバータ非線形性補償:** 電圧モデルベースのオブザーバの精度は、デッドタイム効果やパワーデバイスの電圧降下といったインバータの非線形性によって損なわれます。これらの影響は、指令電圧と実際にモータに印加される電圧との間に誤差を生じさせます。特に、これらの電圧誤差が印加電圧全体に占める割合が大きくなる低速域において、高精度な磁束・トルク推定を達成するためには、これらの非線形性をモデル化し補償することが極めて重要です 6。文献 6 では、この補償がトルク推定精度を劇的に改善することが実験的に検証されています。  
* **離散化とサンプリングの影響:** デジタル実装において、離散化手法（例：前進オイラー法）の選択や制御器のサンプリング時間は、特に電気角がサンプル間で大きく変化する高速域で誤差を導入する可能性があります。これはオブザーバの安定性と精度に影響を与えます。一部の研究では、これらの離散化誤差に明示的に対処しています 12。  
* **ロバスト性とフォールトトレランス:** システム全体のロバスト性も重要です。ハイブリッドオブザーバ 53 は、一方のモデルの故障や不正確さを他方が補償できるため、本質的によりロバストです。LPVやSMOといった手法は、パラメータの不確かさに対するロバスト性を目指して明示的に設計されています 40。

### **4.3. 将来の研究開発動向**

* **高度なAI/MLの統合:** PINN 71 やその他の先進的なNNアーキテクチャ 68 への移行は、研究の最前線です。これらの手法は、空間的に不均一な温度分布 15 のような、モデル化が困難な複雑な物理現象を高忠実度で捉える可能性を秘めています。  
* **自己コミッショニングと自動チューニング:** 最小限の事前情報で、必要なすべての電気的・熱的パラメータをオンラインで自動的に同定できるアルゴリズムの開発が進んでいます。これにより、エンジニアリング工数が大幅に削減され、経年劣化や個体差に適応できるドライブが実現します。RLSやEKFは、この方向への一歩です 34。  
* **統合された電気・熱共同設計:** 究極の目標は、設計後の補償から脱却し、これらの高精度な熱モデルをモータの設計段階そのもので活用することです。これにより、モータの形状や材料を、熱性能と制御のロバスト性の両面から同時に最適化することが可能になります。

## **結論**

本レポートでは、IPMSMのトルク制御における温度の影響を克服するための、磁束推定モデルとトルク補償戦略について包括的なレビューを行いました。分析から、以下の重要な結論が導き出されます。

1. **温度問題の多面性:** 温度上昇は、単にPM磁束ψfを減少させるだけでなく、固定子抵抗Rsを増加させ、さらには磁気飽和を介してインダクタンスLd, Lqにも影響を及ぼす、相互に関連した複雑な問題です。これらの変動は、MTPA制御や弱め磁束制御といった最適制御軌道をドリフトさせ、トルク精度と運転効率を著しく低下させます。  
2. **推定手法の選択はトレードオフ:** オンラインでのパラメータ推定には、電気モデルベース、熱モデルベース、信号注入法、AIベースなど多様なアプローチが存在します。単一の万能な手法はなく、選択はアプリケーションの性能要求、計算機リソース、そして運転速度範囲といった要求仕様間のトレードオフによって決定されます。  
   * **高精度・広範囲:** 最高の性能が求められる場合、応答の速い電気オブザーバ（例：SOGIベース磁束オブザーバ）と安定した熱モデル（LPTN）を組み合わせた**ハイブリッドオブザーバ**が最もロバストな解を提供します。これは、動的な応答性と長期的な安定性を両立できるためです。  
   * **低速・停止域:** ゼロ速を含む低速域での運転が必須の場合、逆起電力ベースのオブザーバは機能しないため、**信号注入法**の採用が不可欠です。この場合、ノイズやトルクリップルを低減する工夫と、中高速域のオブザーバへの滑らかな移行制御が設計の鍵となります。  
3. **補償は全速度域で統一的に:** トルク補償は、MTPA制御領域と弱め磁束制御領域の両方で一貫して行われる必要があります。推定された温度・磁束パラメータは、両方の制御ロジックに供給され、全速度域にわたって最適な電流指令値を生成するために使用されるべきです。  
4. **実装上の鍵:** 理論的なモデルの選択に加え、インバータの非線形性補償や離散化誤差への配慮といった実装レベルの詳細が、最終的なシステムの精度を大きく左右します。特に、電圧モデルベースの手法を用いる場合、インバータのデッドタイム補償は必須と考えるべきです。

推奨事項:  
ユーザの目的である「出力トルクの補償」、特に「モータの温度特性への対応」をFOC、MTPA、MPP（弱め磁束）制御下で実現するためには、以下の段階的なアプローチを推奨します。

1. **ベースラインの構築:** まず、中高速域向けに**SOGIベースの磁束オブザーバ**を実装します。これは、LPFベースの手法よりもDCオフセット除去能力に優れ、EKFよりも計算負荷が低い、バランスの取れた選択肢です。このオブザーバから推定したψfを用いて、MTPA方程式をオンラインで再計算し、弱め磁束制御では電圧フィードバックループを適応させることで、基本的な温度補償を実現します。  
2. **低速性能の確保:** 次に、低速・停止域での運転が必要な場合、**高周波信号注入法**を追加で実装します。注入信号に起因するノイズとリップルを抑制するため、擬似ランダム信号やPWM周波数と同期した注入など、最新の低減技術を検討します。そして、信号注入法と磁束オブザーバ法の間を、速度に応じた重み付け関数などを用いて滑らかに移行させるハイブリッド制御ロジックを構築します。  
3. **ロバスト性の向上:** 最高のロバスト性と精度を追求する場合、上記システムに**LPTN（集中定数熱回路網）を統合したハイブリッド電気・熱オブザーバ**へと拡張します。LPTNは、損失モデルの構築とパラメータ同定が必要ですが、システムの長期的な安定性を大幅に向上させ、電気的励起が不十分な状況でも温度推定を継続できるという大きな利点があります。

この段階的なアプローチにより、開発リソースと性能要求のバランスを取りながら、ロバストで高精度な温度適応型トルク制御システムの構築が可能となります。将来的には、PINNなどのAI技術が、これらの物理モデルベースの手法をさらに高度化し、より複雑な現象まで捉えることができるようになると期待されます。

#### **引用文献**

1. Permanent Magnet Flux Linkage Analysis and Maximum Torque per Ampere (MTPA) Control of High Saturation IPMSM \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/16/12/4717](https://www.mdpi.com/1996-1073/16/12/4717)  
2. No.5 IPMモータってどんなモータ？ \- オリムベクスタ, 6月 19, 2025にアクセス、 [https://www.orimvexta.co.jp/support/specialcontents/no5/](https://www.orimvexta.co.jp/support/specialcontents/no5/)  
3. 1\. まえがき 2\. 本開発IPMモータの主要諸元 3\. トルク特性, 6月 19, 2025にアクセス、 [https://www.sanyodenki.com/archive/document/corporatedata/technicalreport/2001/TR11j\_h\_jp.pdf](https://www.sanyodenki.com/archive/document/corporatedata/technicalreport/2001/TR11j_h_jp.pdf)  
4. Understanding permanent magnet motors \- Control Engineering, 6月 19, 2025にアクセス、 [https://www.controleng.com/understanding-permanent-magnet-motors/](https://www.controleng.com/understanding-permanent-magnet-motors/)  
5. Advanced Torque Control of Interior Permanent Magnet Motors for Electrical Hypercars, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2032-6653/15/2/46](https://www.mdpi.com/2032-6653/15/2/46)  
6. A Practical Torque Estimation Method for Interior Permanent Magnet ..., 6月 19, 2025にアクセス、 [https://pmc.ncbi.nlm.nih.gov/articles/PMC4482678/](https://pmc.ncbi.nlm.nih.gov/articles/PMC4482678/)  
7. Review of MTPA Control of Permanent Magnet Synchronous Motor Considering Parameter Uncertainties, 6月 19, 2025にアクセス、 [https://epjournal.csee.org.cn/zgdjgcxb/en/article/doi/10.13334/j.0258-8013.pcsee.202564](https://epjournal.csee.org.cn/zgdjgcxb/en/article/doi/10.13334/j.0258-8013.pcsee.202564)  
8. Identification of Three Phase IPM Machine Parameters Using ..., 6月 19, 2025にアクセス、 [https://nottingham-repository.worktribe.com/file/831893/1/Identification%20of%20Three%20Phase%20IPM%20Machine%20Parameters%20Using%20Torque%20Tests.pdf](https://nottingham-repository.worktribe.com/file/831893/1/Identification%20of%20Three%20Phase%20IPM%20Machine%20Parameters%20Using%20Torque%20Tests.pdf)  
9. Online parameter estimation for permanent magnet synchronous machines : an overview, 6月 19, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf](https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf)  
10. MTPA control of IPMSM drives based on virtual signal injection considering machine parameter variations, 6月 19, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/126176/1/Tianfu\_Sun\_17-TIE-2293.pdf](https://eprints.whiterose.ac.uk/id/eprint/126176/1/Tianfu_Sun_17-TIE-2293.pdf)  
11. Torque Feed-Back MTPA Control for IPMSM Compensating for Magnet Flux Variation Due to Permanent Magnet Temperature | Request PDF \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/347869834\_Torque\_Feed-Back\_MTPA\_Control\_for\_IPMSM\_Compensating\_for\_Magnet\_Flux\_Variation\_Due\_to\_Permanent\_Magnet\_Temperature](https://www.researchgate.net/publication/347869834_Torque_Feed-Back_MTPA_Control_for_IPMSM_Compensating_for_Magnet_Flux_Variation_Due_to_Permanent_Magnet_Temperature)  
12. (PDF) Observer for the rotor temperature of IPMSM \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/224184336\_Observer\_for\_the\_rotor\_temperature\_of\_IPMSM](https://www.researchgate.net/publication/224184336_Observer_for_the_rotor_temperature_of_IPMSM)  
13. Comparative Study of Magnet Temperature Estimation at Low Speeds Based on High-Frequency Resistance and Inductance \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2079-9292/12/9/2011](https://www.mdpi.com/2079-9292/12/9/2011)  
14. Non-linear Optimization-Based Temperature Estimation of IPMSM \- MPLab, 6月 19, 2025にアクセス、 [https://mplab.ee.columbia.edu/sites/default/files/content/Publications/Sun2019%20-%20Non-Linear%20Optimization-Based%20Temperature%20Estimation%20of%20IPMSM.pdf](https://mplab.ee.columbia.edu/sites/default/files/content/Publications/Sun2019%20-%20Non-Linear%20Optimization-Based%20Temperature%20Estimation%20of%20IPMSM.pdf)  
15. Online temperature estimation of IPMSM permanent magnets in hybrid electric vehicles | Request PDF \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/252026119\_Online\_temperature\_estimation\_of\_IPMSM\_permanent\_magnets\_in\_hybrid\_electric\_vehicles](https://www.researchgate.net/publication/252026119_Online_temperature_estimation_of_IPMSM_permanent_magnets_in_hybrid_electric_vehicles)  
16. \[JAC296\] IPMモータの温度分布と温度変化を考慮したトルク特性解析 | 電磁界解析ソフトウェア, 6月 19, 2025にアクセス、 [https://www.jmag-international.com/jp/catalog/296\_ipm\_torquetemperature/](https://www.jmag-international.com/jp/catalog/296_ipm_torquetemperature/)  
17. 2022年に読んだモータ・パワエレ・機械学習関連の論文まとめ, 6月 19, 2025にアクセス、 [https://yuyumoyuyu.com/2022/01/05/paper2022/](https://yuyumoyuyu.com/2022/01/05/paper2022/)  
18. Temperature Estimation of IPMSM Using Thermal Equivalent Circuit \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/258656738\_Temperature\_Estimation\_of\_IPMSM\_Using\_Thermal\_Equivalent\_Circuit](https://www.researchgate.net/publication/258656738_Temperature_Estimation_of_IPMSM_Using_Thermal_Equivalent_Circuit)  
19. Active thermal management for Interior Permanent Magnet Synchronous Machine (IPMSM) drives based on model predictive control \- White Rose Research Online, 6月 19, 2025にアクセス、 [https://eprints.whiterose.ac.uk/id/eprint/131330/8/MPC\_paper\_final\_NC.pdf](https://eprints.whiterose.ac.uk/id/eprint/131330/8/MPC_paper_final_NC.pdf)  
20. A Super-Twisting Sliding-Mode Stator Flux Observer for Sensorless Direct Torque and Flux Control of IPMSM \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/12/13/2564](https://www.mdpi.com/1996-1073/12/13/2564)  
21. MTPA Control for IPMSM Drives Based on Pseudorandom Frequency-Switching Sinusoidal Signal Injection \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2075-1702/10/4/231](https://www.mdpi.com/2075-1702/10/4/231)  
22. A Torque Compensation Control Scheme of PMSM considering a Wide Variation of Permanent Magnet Temperature \- International Compumag Society, 6月 19, 2025にアクセス、 [https://www.compumag.org/Proceedings/2017\_Daejeon/papers/\[PD-M2-2\]\_252.pdf](https://www.compumag.org/Proceedings/2017_Daejeon/papers/[PD-M2-2]_252.pdf)  
23. 産業・車載用永久磁石モータ及びドライブ装置 \- 東芝, 6月 19, 2025にアクセス、 [https://www.global.toshiba/content/dam/toshiba/migration/corp/techReviewAssets/tech/review/2000/07/a12.pdf](https://www.global.toshiba/content/dam/toshiba/migration/corp/techReviewAssets/tech/review/2000/07/a12.pdf)  
24. MTPA Control Reference \- Compute reference currents for maximum torque per ampere (MTPA) and field-weakening operation \- Simulink \- MathWorks, 6月 19, 2025にアクセス、 [https://www.mathworks.com/help/mcb/ref/mtpacontrolreference.html](https://www.mathworks.com/help/mcb/ref/mtpacontrolreference.html)  
25. Torque Control Strategy of an IPMSM Considering the Flux Variation of the Permanent Magnet \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/224290184\_Torque\_Control\_Strategy\_of\_an\_IPMSM\_Considering\_the\_Flux\_Variation\_of\_the\_Permanent\_Magnet](https://www.researchgate.net/publication/224290184_Torque_Control_Strategy_of_an_IPMSM_Considering_the_Flux_Variation_of_the_Permanent_Magnet)  
26. Sensorless-FOC With Flux-Weakening and MTPA for IPMSM Motor Drives \- Texas Instruments, 6月 19, 2025にアクセス、 [https://www.ti.com/lit/pdf/spracf3](https://www.ti.com/lit/pdf/spracf3)  
27. A Review about Flux-Weakening Operating Limits and Control Techniques for Synchronous Motor Drives \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/15/5/1930](https://www.mdpi.com/1996-1073/15/5/1930)  
28. PMSM Drive Characteristics and Constraint Curves \- MATLAB & Simulink \- MathWorks, 6月 19, 2025にアクセス、 [https://www.mathworks.com/help/mcb/ug/pmsm-characteristics-constraint-curves.html](https://www.mathworks.com/help/mcb/ug/pmsm-characteristics-constraint-curves.html)  
29. Current limit circle, voltage limit ellipse, and constant torque curve on d-q axes., 6月 19, 2025にアクセス、 [https://www.researchgate.net/figure/Current-limit-circle-voltage-limit-ellipse-and-constant-torque-curve-on-d-q-axes\_fig2\_260496685](https://www.researchgate.net/figure/Current-limit-circle-voltage-limit-ellipse-and-constant-torque-curve-on-d-q-axes_fig2_260496685)  
30. US8519648B2 \- Temperature compensation for improved field weakening accuracy \- Google Patents, 6月 19, 2025にアクセス、 [https://patents.google.com/patent/US8519648B2/en](https://patents.google.com/patent/US8519648B2/en)  
31. Improved flux linkage observer for position estimation of permanent magnet synchronous linear motor \- Recent, 6月 19, 2025にアクセス、 [https://ms.copernicus.org/articles/15/99/2024/](https://ms.copernicus.org/articles/15/99/2024/)  
32. Optimizing Sensorless Control in PMSM Based on the SOGIFO-X Flux Observer Algorithm, 6月 19, 2025にアクセス、 [https://pmc.ncbi.nlm.nih.gov/articles/PMC10857136/](https://pmc.ncbi.nlm.nih.gov/articles/PMC10857136/)  
33. Research on TOGI‐EFLL flux observer for IPMSM sensorless control ..., 6月 19, 2025にアクセス、 [https://digital-library.theiet.org/doi/full/10.1049/pel2.12735](https://digital-library.theiet.org/doi/full/10.1049/pel2.12735)  
34. An Online Parameter Estimation Using Current Injection with Intelligent Current-Loop Control for IPMSM Drives \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/14/23/8138](https://www.mdpi.com/1996-1073/14/23/8138)  
35. Performance Improvement of Sensorless IPMSM Drives in a Low-Speed Region Using Online Parameter Identification \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/224207534\_Performance\_Improvement\_of\_Sensorless\_IPMSM\_Drives\_in\_a\_Low-Speed\_Region\_Using\_Online\_Parameter\_Identification](https://www.researchgate.net/publication/224207534_Performance_Improvement_of_Sensorless_IPMSM_Drives_in_a_Low-Speed_Region_Using_Online_Parameter_Identification)  
36. Dependence of IPMSM Motor Efficiency on Parameter Estimates \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2071-1050/13/16/9299](https://www.mdpi.com/2071-1050/13/16/9299)  
37. Parameter identification of PMSM using EKF with temperature variation tracking in automotive applications \- SciSpace, 6月 19, 2025にアクセス、 [https://scispace.com/pdf/parameter-identification-of-pmsm-using-ekf-with-temperature-3lwy82jjzh.pdf](https://scispace.com/pdf/parameter-identification-of-pmsm-using-ekf-with-temperature-3lwy82jjzh.pdf)  
38. On-line Parameter Estimation of Interior Permanent Magnet Synchronous Motor using an Extended Kalman Filter \- Korea Science, 6月 19, 2025にアクセス、 [https://www.koreascience.kr/article/JAKO201407651680963.page?\&lang=ko](https://www.koreascience.kr/article/JAKO201407651680963.page?&lang=ko)  
39. Compensation Algorithms for Sliding Mode Observers in Sensorless Control of IPMSMs \- UNL Digital Commons, 6月 19, 2025にアクセス、 [https://digitalcommons.unl.edu/cgi/viewcontent.cgi?article=1199\&context=electricalengineeringfacpub](https://digitalcommons.unl.edu/cgi/viewcontent.cgi?article=1199&context=electricalengineeringfacpub)  
40. (PDF) Robust flux observer and robust block controller design for interior permanent magnet synchronous motor under demagnetisation fault \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/328069412\_Robust\_flux\_observer\_and\_robust\_block\_controller\_design\_for\_interior\_permanent\_magnet\_synchronous\_motor\_under\_demagnetisation\_fault](https://www.researchgate.net/publication/328069412_Robust_flux_observer_and_robust_block_controller_design_for_interior_permanent_magnet_synchronous_motor_under_demagnetisation_fault)  
41. Compensating Thermal Derated Torque of IPMSM Centric Electric Vehicles \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/358965842\_Compensating\_Thermal\_Derated\_Torque\_of\_IPMSM\_Centric\_Electric\_Vehicles](https://www.researchgate.net/publication/358965842_Compensating_Thermal_Derated_Torque_of_IPMSM_Centric_Electric_Vehicles)  
42. Compensating Thermal Derated Torque of IPMSM Centric Electric Vehicles \- SciSpace, 6月 19, 2025にアクセス、 [https://scispace.com/pdf/compensating-thermal-derated-torque-of-ipmsm-centric-25g75e3d.pdf](https://scispace.com/pdf/compensating-thermal-derated-torque-of-ipmsm-centric-25g75e3d.pdf)  
43. Regression Model-Based Flux Observer for IPMSM Sensorless Control with Wide Speed Range \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/14/19/6249](https://www.mdpi.com/1996-1073/14/19/6249)  
44. Regression Model-Based Flux Observer for IPMSM Sensorless Control with Wide Speed Range \- PSE Community.org, 6月 19, 2025にアクセス、 [https://psecommunity.org/wp-content/plugins/wpor/includes/file/2303/LAPSE-2023.19054-1v1.pdf](https://psecommunity.org/wp-content/plugins/wpor/includes/file/2303/LAPSE-2023.19054-1v1.pdf)  
45. A Simplified Thermal Model and Online Temperature Estimation Method of Permanent Magnet Synchronous Motors \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2076-3417/9/15/3158](https://www.mdpi.com/2076-3417/9/15/3158)  
46. Real-Time Rotor Temperature Estimation Method for Interior Permanent Magnet Synchronous Machines \- DiVA portal, 6月 19, 2025にアクセス、 [https://www.diva-portal.org/smash/get/diva2:1737838/FULLTEXT01.pdf](https://www.diva-portal.org/smash/get/diva2:1737838/FULLTEXT01.pdf)  
47. Estimation of the Magnet Temperature via a Lumped Parameter Thermal Network in Real Time for the Control of PMSM \- Teaching and Research Area Mechatronics in Mobile Propulsion \- RWTH Aachen University, 6月 19, 2025にアクセス、 [https://www.mmp.rwth-aachen.de/global/show\_document.asp?id=aaaaaaaabqfgkxh\&download=1](https://www.mmp.rwth-aachen.de/global/show_document.asp?id=aaaaaaaabqfgkxh&download=1)  
48. Development of a Fast Thermal Model for Calculating the Temperature of the Interior PMSM, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/14/22/7455](https://www.mdpi.com/1996-1073/14/22/7455)  
49. Validation and Parametric Investigations Using a Lumped Thermal Parameter Model of an Internal Permanent Magnet Motor \- Publications, 6月 19, 2025にアクセス、 [https://docs.nrel.gov/docs/fy21osti/77082.pdf](https://docs.nrel.gov/docs/fy21osti/77082.pdf)  
50. (PDF) Sensitivity Analysis and Uncertainty Quantification of Thermal Behaviour for Outer Rotor Permanent Magnet Synchronous Machines \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/380456813\_Sensitivity\_Analysis\_and\_Uncertainty\_Quantification\_of\_Thermal\_Behaviour\_for\_Outer\_Rotor\_Permanent\_Magnet\_Synchronous\_Machines](https://www.researchgate.net/publication/380456813_Sensitivity_Analysis_and_Uncertainty_Quantification_of_Thermal_Behaviour_for_Outer_Rotor_Permanent_Magnet_Synchronous_Machines)  
51. A Model-Based Lumped Parameter Thermal Network for Online Temperature Estimation of IPMSM in Automotive Applications \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/379281371\_A\_Model-Based\_Lumped\_Parameter\_Thermal\_Network\_for\_Online\_Temperature\_Estimation\_of\_IPMSM\_in\_Automotive\_Applications](https://www.researchgate.net/publication/379281371_A_Model-Based_Lumped_Parameter_Thermal_Network_for_Online_Temperature_Estimation_of_IPMSM_in_Automotive_Applications)  
52. Thermal Parametric Sensitivity Analysis of an IPMSM With Multi Three-Phase Sector Windings Topology Under Normal, Partial and Partial Overload Operating Conditions | Request PDF \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/357443820\_Thermal\_Parametric\_Sensitivity\_Analysis\_of\_an\_IPMSM\_With\_Multi\_Three-Phase\_Sector\_Windings\_Topology\_Under\_Normal\_Partial\_and\_Partial\_Overload\_Operating\_Conditions](https://www.researchgate.net/publication/357443820_Thermal_Parametric_Sensitivity_Analysis_of_an_IPMSM_With_Multi_Three-Phase_Sector_Windings_Topology_Under_Normal_Partial_and_Partial_Overload_Operating_Conditions)  
53. Online Approach for Parameter Identification and Temperature Estimation in Permanent Magnet Synchronous Machines, 6月 19, 2025にアクセス、 [https://d-nb.info/128171318X/34](https://d-nb.info/128171318X/34)  
54. Online Approach for Parameter Identification and Temperature Estimation in Permanent Magnet Synchronous Machines \- mediaTUM, 6月 19, 2025にアクセス、 [https://mediatum.ub.tum.de/doc/1638373/1638373.pdf](https://mediatum.ub.tum.de/doc/1638373/1638373.pdf)  
55. On-line Temperature Monitoring of Permanent Magnet Synchronous Machines Shuai Xiao, BEng., MSc \- White Rose eTheses Online, 6月 19, 2025にアクセス、 [https://etheses.whiterose.ac.uk/id/eprint/24936/1/Thesis\_SXiao\_WhiteRose.pdf](https://etheses.whiterose.ac.uk/id/eprint/24936/1/Thesis_SXiao_WhiteRose.pdf)  
56. Monitoring the Thermal Condition of Permanent-Magnet Synchronous Motors \- Aerospace and Electronic Systems, IEEE Transactions on, 6月 19, 2025にアクセス、 [https://users.soe.ucsc.edu/\~milanfar/publications/journal/aes96.pdf](https://users.soe.ucsc.edu/~milanfar/publications/journal/aes96.pdf)  
57. A new signal injection-based method for estimation of position in interior permanent magnet synchronous motors, 6月 19, 2025にアクセス、 [https://par.nsf.gov/servlets/purl/10465782](https://par.nsf.gov/servlets/purl/10465782)  
58. Sensorless Control Strategy for Interior Permanent Magnet Synchronous Motors in the Full-Speed Section \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/16/23/7701](https://www.mdpi.com/1996-1073/16/23/7701)  
59. Int'l Journal \- EEPEL | SNU Electrical Engineering & Power Electronics Lab, 6月 19, 2025にアクセス、 [http://eepel.snu.ac.kr/wordpress/international-journal/](http://eepel.snu.ac.kr/wordpress/international-journal/)  
60. New Acoustic Noise Reduction Method for Signal-Injection-Based IPMSM Sensorless Drive, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/365968382\_New\_Acoustic\_Noise\_Reduction\_Method\_for\_Signal-Injection-Based\_IPMSM\_Sensorless\_Drive](https://www.researchgate.net/publication/365968382_New_Acoustic_Noise_Reduction_Method_for_Signal-Injection-Based_IPMSM_Sensorless_Drive)  
61. Reduction of high-frequency injection losses, acoustic noise and total harmonic distortion in IPMSM sensorless drives | IET Power Electronics, 6月 19, 2025にアクセス、 [https://digital-library.theiet.org/doi/full/10.1049/iet-pel.2018.6250?doi=10.1049/iet-pel.2018.6250](https://digital-library.theiet.org/doi/full/10.1049/iet-pel.2018.6250?doi=10.1049/iet-pel.2018.6250)  
62. Noise Reduction in an Interior Permanent Magnet Synchronous Motor through Structural Modifications | Request PDF \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/383563073\_Noise\_Reduction\_in\_an\_Interior\_Permanent\_Magnet\_Synchronous\_Motor\_through\_Structural\_Modifications](https://www.researchgate.net/publication/383563073_Noise_Reduction_in_an_Interior_Permanent_Magnet_Synchronous_Motor_through_Structural_Modifications)  
63. Random-Phase Signal Injection Combined With Random PWM for Low-Noise IPMSM Sensorless Drives \- Le Biblioteche \- PICO \- Polito, 6月 19, 2025にアクセス、 [https://pico.polito.it/discovery/fulldisplay?docid=cdi\_ieee\_primary\_10567212\&context=PC\&vid=39PTO\_INST:VU\&lang=it\&search\_scope=MyInst\_and\_CI\&adaptor=Primo%20Central\&tab=Everything\&query=sub%2Cexact%2Chigh-frequency%20square-wave%20voltage%20injection%2CAND\&mode=advanced\&offset=0](https://pico.polito.it/discovery/fulldisplay?docid=cdi_ieee_primary_10567212&context=PC&vid=39PTO_INST:VU&lang=it&search_scope=MyInst_and_CI&adaptor=Primo+Central&tab=Everything&query=sub,exact,high-frequency+square-wave+voltage+injection,AND&mode=advanced&offset=0)  
64. Wide-Speed-Range Sensorless Control of IPMSM \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2079-9292/11/22/3747](https://www.mdpi.com/2079-9292/11/22/3747)  
65. Smooth Transition of Multi-mode Synchronous Modulation for IPMSM Sensorless Drives in Rail-Transit Applications \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/338444047\_Smooth\_Transition\_of\_Multi-mode\_Synchronous\_Modulation\_for\_IPMSM\_Sensorless\_Drives\_in\_Rail-Transit\_Applications](https://www.researchgate.net/publication/338444047_Smooth_Transition_of_Multi-mode_Synchronous_Modulation_for_IPMSM_Sensorless_Drives_in_Rail-Transit_Applications)  
66. Smooth Transition of Multimode Synchronous Modulation for IPMSM Sensorless Drives in Rail-Transit Applications \- X-MOL, 6月 19, 2025にアクセス、 [https://m.x-mol.net/paper/detail/1320893358477119488](https://m.x-mol.net/paper/detail/1320893358477119488)  
67. Hybrid Position-Sensorless Control Scheme for PMSM Based on Combination of IF Control and Sliding Mode Observer \- 电工技术学报, 6月 19, 2025にアクセス、 [https://dgjsxb.ces-transaction.com/EN/abstract/abstract5038.shtml](https://dgjsxb.ces-transaction.com/EN/abstract/abstract5038.shtml)  
68. Predicting the Temperature of a Permanent Magnet Synchronous Motor: A Comparative Study of Artificial Neural Network Algorithms \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2227-7080/13/3/120](https://www.mdpi.com/2227-7080/13/3/120)  
69. Temperature Estimation of PMSM Using a Difference-Estimating Feedforward Neural Network \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/342973806\_Temperature\_Estimation\_of\_PMSM\_Using\_a\_Difference-Estimating\_Feedforward\_Neural\_Network](https://www.researchgate.net/publication/342973806_Temperature_Estimation_of_PMSM_Using_a_Difference-Estimating_Feedforward_Neural_Network)  
70. End-to-End Differentiable Physics Temperature Estimation for Permanent Magnet Synchronous Motor \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/2032-6653/15/4/174](https://www.mdpi.com/2032-6653/15/4/174)  
71. PINNverse: Accurate parameter estimation in differential equations from noisy data with constrained physics-informed neural networks \- arXiv, 6月 19, 2025にアクセス、 [https://arxiv.org/pdf/2504.05248](https://arxiv.org/pdf/2504.05248)  
72. \[2504.05248\] PINNverse: Accurate parameter estimation in differential equations from noisy data with constrained physics-informed neural networks \- arXiv, 6月 19, 2025にアクセス、 [https://arxiv.org/abs/2504.05248](https://arxiv.org/abs/2504.05248)  
73. Parameter Identification of Permanent Magnet Synchronous Motor Based on Physics-Informed Neural Network \- ResearchGate, 6月 19, 2025にアクセス、 [https://www.researchgate.net/publication/390217765\_Parameter\_Identification\_of\_Permanent\_Magnet\_Synchronous\_Motor\_Based\_on\_Physics-Informed\_Neural\_Network](https://www.researchgate.net/publication/390217765_Parameter_Identification_of_Permanent_Magnet_Synchronous_Motor_Based_on_Physics-Informed_Neural_Network)  
74. Physics Informed Neural Net Model for The Parameter Estimation of EV Chargers \- TU Delft, 6月 19, 2025にアクセス、 [https://www.tudelft.nl/en/ese/dces/physics-informed-neural-net-model-for-the-parameter-estimation-of-ev-chargers](https://www.tudelft.nl/en/ese/dces/physics-informed-neural-net-model-for-the-parameter-estimation-of-ev-chargers)  
75. Auto-Optimized Maximum Torque Per Ampere Control of IPMSM Using Dual Control for Exploration and Exploitation This work was supported in part by the UK Engineering and Physical Science Research Council (EPSRC) New Investigator Award under Grant EP/W027283/1. Dr. Jun Yang is the corresponding author. Emails: {y.zuo, y \- arXiv, 6月 19, 2025にアクセス、 [https://arxiv.org/html/2404.18176v1](https://arxiv.org/html/2404.18176v1)  
76. Novel Flux-weakening Control of Permanent Magnet Synchronous Machines with Particular Reference to Stability Issues Chao Wang, 6月 19, 2025にアクセス、 [https://etheses.whiterose.ac.uk/id/eprint/25058/1/Novel%20Flux-weakening%20Control%20of%20PMSM%20with%20Particular%20Reference%20to%20Stability%20Issues-submitted.pdf](https://etheses.whiterose.ac.uk/id/eprint/25058/1/Novel%20Flux-weakening%20Control%20of%20PMSM%20with%20Particular%20Reference%20to%20Stability%20Issues-submitted.pdf)  
77. Flux-Weakening Drive for IPMSM Based on Model Predictive Control \- MDPI, 6月 19, 2025にアクセス、 [https://www.mdpi.com/1996-1073/15/7/2543](https://www.mdpi.com/1996-1073/15/7/2543)  
78. Sensorless Control of Ultra-High-Speed PMSM via Improved PR and Adaptive Position Observer \- PMC, 6月 19, 2025にアクセス、 [https://pmc.ncbi.nlm.nih.gov/articles/PMC11902340/](https://pmc.ncbi.nlm.nih.gov/articles/PMC11902340/)