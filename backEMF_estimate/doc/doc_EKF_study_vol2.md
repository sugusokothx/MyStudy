

# **拡張カルマンフィルタ：理論、柔軟性、および比較性能に関する包括的分析**

## **第1章 基盤：線形カルマンフィルタ（KF）**

拡張カルマンフィルタ（EKF）の革新性を理解するためには、まずその前身である線形カルマンフィルタ（KF）の理論的基盤と限界を深く掘り下げる必要があります。KFは、特定の理想的な条件下で、数学的に最適な推定器として機能し、その後の多くのフィルタリング技術の礎を築きました。

### **1.1. 状態空間パラダイム：動的システムの記述言語**

カルマンフィルタの理論は、動的システムを記述するための強力な数学的フレームワークである「状態空間表現」に基づいています 1。このアプローチでは、システムの振る舞いを2つの方程式でモデル化します。

1. **状態方程式（プロセスモデル）**：この方程式は、システムの内部状態が時間とともにどのように進化するかを記述します。直接観測することはできないかもしれませんが、システムの本質的な特性（例：物体の位置、速度、姿勢）を含んでいます。一般的に、離散時間システムの状態方程式は次のように表されます 3。

   $$ \\boldsymbol{x}k \= \\boldsymbol{F} \\boldsymbol{x}{k-1} \+ \\boldsymbol{G} \\boldsymbol{u}{k-1} \+ \\boldsymbol{w}{k-1} $$  
   ここで、xk​は時刻kにおける状態ベクトル、$\\boldsymbol{x}{k-1}は1ステップ前の状態ベクトルです。\\boldsymbol{F}は状態遷移行列と呼ばれ、前の状態から現在の状態への遷移を線形に表現します。\\boldsymbol{u}{k-1}は制御入力ベクトル（例：モータへの指令値）、\\boldsymbol{G}は入力係数行列です。そして、\\boldsymbol{w}\_{k-1}$はシステムノイズ（またはプロセスノイズ）であり、モデル化しきれない不確実性や外乱を表します。  
2. 観測方程式（測定モデル）：この方程式は、直接観測できない内部状態$\\boldsymbol{x}\_kと、センサーなどによって実際に測定される観測値\\boldsymbol{z}\_k$との関係を記述します 3。  
   $$\\boldsymbol{z}\_k \= \\boldsymbol{H} \\boldsymbol{x}\_k \+ \\boldsymbol{v}\_k$$  
   ここで、$\\boldsymbol{H}は観測行列と呼ばれ、状態ベクトルを観測空間に線形に写像します。\\boldsymbol{v}\_k$は観測ノイズ（または測定ノイズ）であり、センサー自身の不正確さや環境ノイズを表します。

カルマンフィルタの根源的な目的は、ノイズを含んだ観測値$\\boldsymbol{z}の時系列データを用いて、直接観測できない内部状態\\boldsymbol{x}$を可能な限り正確に推定することにあります 5。

### **1.2. 中核となる原理と仮定：理想化されたカルマンの世界**

KFがその数学的な最適性を発揮するためには、いくつかの厳格な前提条件を満たす必要があります 6。これらの仮定を理解することは、なぜEKFが必要とされるのかを理解する上で不可欠です。

* **線形性の仮定**：最も重要な制約は、システムが線形であることです。つまり、状態方程式における状態遷移（行列$\\boldsymbol{F}）と、観測方程式における観測プロセス（行列\\boldsymbol{H}$）が、共に線形変換でなければなりません 7。現実世界の多くのシステムは非線形な振る舞いを示すため、これがKFの適用範囲を限定する最大の要因となります。  
* **ガウスノイズの仮定**：システムノイズ$\\boldsymbol{w}と観測ノイズ\\boldsymbol{v}は、平均が0である正規分布（ガウス分布）に従うと仮定されます\[4,9\]。これらのノイズの統計的性質は、それぞれ共分散行列\\boldsymbol{Q}と\\boldsymbol{R}$によって完全に記述され、これらの値は既知であるとされます。  
* **無相関ノイズの仮定**：システムノイズと観測ノイズは、互いに無相関（統計的に独立）であると仮定されます 4。つまり、一方のノイズが他方のノイズに影響を与えることはありません。

これらの仮定が満たされるとき、カルマンフィルタは線形システムに対する最小分散推定器、すなわち考えられるすべての線形推定器の中で最も誤差が小さい、最適な推定器となります。

### **1.3. 2段階のアルゴリズム：予測と更新の再帰的なダンス**

カルマンフィルタは、過去のすべてのデータを保持する必要がなく、直前の推定値のみを用いて現在の状態を推定する「再帰的」なアルゴリズムです 10。この効率的な構造は、「予測」と「更新」という2つのステップを繰り返すことで実現されます。

* **予測ステップ（時間更新）**：このステップでは、システムモデル（状態方程式）を用いて、次の観測値が得られる前の状態を予測します 3。  
  1. 状態の事前推定：直前の状態推定値$\\boldsymbol{\\hat{x}}\_{k-1}を用いて、現在の状態\\boldsymbol{\\hat{x}}k^-$を予測します。（上付きのマイナス記号は、観測値による更新前の「事前」推定値であることを示します。）  
     $$ \\boldsymbol{\\hat{x}}k^- \= \\boldsymbol{F} \\boldsymbol{\\hat{x}}{k-1} \+ \\boldsymbol{G} \\boldsymbol{u}{k-1} $$  
  2. 誤差共分散の事前推定：状態推定の不確かさを表す誤差共分散行列$\\boldsymbol{P}$も同様に予測します。  
     Pk−​=FPk−1​FT+Q

     このステップは、システムダイナミクスに基づいて、状態とその不確かさが時間とともにどのように伝播するかを計算するプロセスです。  
* **更新ステップ（観測更新）**：新しい観測値$\\boldsymbol{z}\_k$が得られると、この情報を使って予測ステップで得られた事前推定値を「補正」します 3。  
  1. カルマンゲインの計算：予測の不確かさ（Pk−​）と観測の不確かさ（R）を比較し、どちらをどれだけ信用するかを決定する重み付け係数、カルマンゲイン$\\boldsymbol{K}\_k$を計算します。  
     $$ \\boldsymbol{K}\_k \= \\boldsymbol{P}\_k^- \\boldsymbol{H}^T (\\boldsymbol{H} \\boldsymbol{P}\_k^- \\boldsymbol{H}^T \+ \\boldsymbol{R})^{-1} $$  
  2. 状態の事後推定：事前推定値$\\boldsymbol{\\hat{x}}\_k^-を、実際の観測値\\boldsymbol{z}\_kと予測された観測値\\boldsymbol{H}\\boldsymbol{\\hat{x}}\_k^-の差（イノベーションと呼ばれる）を使って更新し、事後推定値\\boldsymbol{\\hat{x}}\_k$を求めます。  
     $$ \\boldsymbol{\\hat{x}}\_k \= \\boldsymbol{\\hat{x}}\_k^- \+ \\boldsymbol{K}\_k (\\boldsymbol{z}\_k \- \\boldsymbol{H} \\boldsymbol{\\hat{x}}\_k^-) $$  
  3. 誤差共分散の事後推定：最後に、観測情報を取り込んだ後の誤差共分散行列$\\boldsymbol{P}\_k$を更新します。  
     Pk​=(I−Kk​H)Pk−​

     この予測と更新のサイクルを時々刻々と繰り返すことで、KFはリアルタイムで状態を追跡し続けることができます。この再帰的な性質こそが、KFをリアルタイム処理に適した手法たらしめているのです 10。

### **1.4. カルマンゲイン（K）：最適な配合比**

カルマンゲイン$\\boldsymbol{K}は、フィルタの心臓部と言えます\[3\]。これは、予測された状態と新しい観測値という2つの情報源を、どのように最適に組み合わせるかを決定する役割を担います。その計算は、事後誤差共分散\\boldsymbol{P}\_k$を最小化するように設計されています 4。

直感的には、カルマンゲインは次のように振る舞います。

* 観測ノイズが大きい（$\\boldsymbol{R}$の値が大きい）場合、カルマンゲインは小さくなります。これは、観測値の信頼性が低いことを意味し、フィルタは予測値をより重視します。  
* 予測の不確かさが大きい（$\\boldsymbol{P}\_k^-$の値が大きい）場合、カルマンゲインは大きくなります。これは、システムモデルに基づく予測の信頼性が低いことを意味し、フィルタは新しい観測値をより重視して推定値を大きく補正します。

このように、カルマンゲインは不確かさに応じて動的に調整され、常に最適なバランスで情報を融合します。このメカニズムは、ベイズ確率論を線形ガウス問題に適用した結果として自然に導出されます。「予測」は事前確率の計算に相当し、「更新」はベイズの定理を用いて観測という新たな証拠を取り込み事後確率を計算するプロセスに相当します。カルマンフィルタは、この特定のベイズフィルタリング問題に対する、エレガントで閉じた形式の解なのです。

### **1.5. 線形世界の境界：なぜKFだけでは不十分なのか**

その強力さにもかかわらず、線形カルマンフィルタの活躍の場は、その厳格な線形性の仮定によって大きく制限されます。ロボットの運動学、航空機の空気力学、化学反応プロセスなど、我々が関心を持つ現実世界のシステムの多くは、本質的に非線形です 8。

これらの非線形システムに線形KFを無理に適用しようとすると、粗雑な近似が必要となり、結果として推定性能が著しく低下したり、最悪の場合は推定値が発散してしまったりします。この根本的な限界が、非線形なダイナミクスを直接扱うことができるフィルタ、すなわち拡張カルマンフィルタ（EKF）の開発を強く動機付けたのです 7。

---

## **第2章 現実への拡張：拡張カルマンフィルタ（EKF）**

線形カルマンフィルタが提供するエレガントな枠組みを、現実世界の大多数を占める非線形システムへ適用するために考案されたのが、拡張カルマンフィルタ（EKF）です。EKFは、KFの直接的かつ直感的な拡張であり、非線形問題に対する状態推定の分野で長年にわたり中心的な役割を果たしてきました。

### **2.1. 非線形ダイナミクスのモデリング**

EKFでは、線形KFの状態空間モデルが、より一般的な非線形関数を用いた形式に置き換えられます 7。

* 非線形状態方程式:  
  xk​=f(xk−1​,uk−1​)+wk−1​  
* 非線形観測方程式:  
  zk​=h(xk​)+vk​

ここで、$\\boldsymbol{f}(\\cdot)と\\boldsymbol{h}(\\cdot)$は、それぞれ状態遷移と観測プロセスを記述する非線形のベクトル関数です。これらの関数は微分可能であることが要求されます 14。この一般化により、例えば、ロボットの車輪の回転数（入力）からその位置と向き（状態）を計算するような複雑な運動学や、ロボットの位置（状態）から特定のランドマークまでの距離と方位（観測値）を計算するようなセンサーの物理モデルを、より忠実に表現することが可能になります 15。

### **2.2. EKFの核心戦略：一次テイラー展開による線形化**

非線形性を導入したことで生じる根本的な問題は、正規分布（ガウス分布）に従う確率変数を非線形関数に通すと、変換後の分布はもはや正規分布ではなくなるという点です 9。平均と共分散だけではこの歪んだ分布を正確に表現できなくなり、KFの数学的な前提が崩れてしまいます。

EKFがこの難問に対して採用する解決策は、非線形関数を特定の点、すなわちその時点での最良の状態推定値の周りで、線形関数に「近似」することです 7。この線形化近似は、関数の一次テイラー展開を用いることで実現されます。これは、関数のある点の近傍では、その点における接線（または接平面）で関数を近似できるという考え方に基づいています。

### **2.3. ヤコビ行列：線形化の道具**

この線形化のプロセスで中心的な役割を果たすのが「ヤコビ行列」です。これは、ベクトル関数のすべての一次偏微分を成分とする行列であり、KFにおける線形な行列$\\boldsymbol{F}と\\boldsymbol{H}$の代替物として機能します 7。

* 状態遷移行列のヤコビアン (Fk​): 非線形状態方程式$\\boldsymbol{f}(\\cdot)を状態ベクトル\\boldsymbol{x}で偏微分し、直前の状態推定値\\boldsymbol{\\hat{x}}{k-1}$で評価したものです。  
  $$ \\boldsymbol{F}k \= \\left. \\frac{\\partial \\boldsymbol{f}}{\\partial \\boldsymbol{x}} \\right|{\\boldsymbol{x}=\\boldsymbol{\\hat{x}}{k-1}, \\boldsymbol{u}=\\boldsymbol{u}\_{k-1}} $$  
* 観測行列のヤコビアン (Hk​): 非線形観測方程式$\\boldsymbol{h}(\\cdot)を状態ベクトル\\boldsymbol{x}で偏微分し、現在の「予測」状態推定値\\boldsymbol{\\hat{x}}\_k^-$で評価したものです。  
  $$ \\boldsymbol{H}k \= \\left. \\frac{\\partial \\boldsymbol{h}}{\\partial \\boldsymbol{x}} \\right|{\\boldsymbol{x}=\\boldsymbol{\\hat{x}}\_k^-} $$

これらのヤコビ行列を解析的に導出し、計算できることがEKFを実装する上での重要な要件となります。これは時に、特にモデルが複雑な場合には、実装上のハードルとなる可能性があります 9。

### **2.4. EKFアルゴリズム詳解**

ヤコビ行列を用いることで、EKFのアルゴリズムは線形KFと非常によく似た構造を保ちます。以下にその詳細なステップを示します 7。

* **予測ステップ**:  
  1. 状態の事前推定：状態の予測には、非線形の状態方程式$\\boldsymbol{f}(\\cdot)$を直接用います。  
     x^k−​=f(x^k−1​,uk−1​)  
  2. 誤差共分散の事前推定：誤差共分散の伝播には、線形化によって得られたヤコビ行列Fk​を用います。  
     Pk−​=Fk​Pk−1​FkT​+Q  
* **更新ステップ**:  
  1. カルマンゲインの計算：カルマンゲインの計算には、ヤコビ行列Hk​を用います。  
     $$ \\boldsymbol{K}\_k \= \\boldsymbol{P}\_k^- \\boldsymbol{H}\_k^T (\\boldsymbol{H}\_k \\boldsymbol{P}\_k^- \\boldsymbol{H}\_k^T \+ \\boldsymbol{R})^{-1} $$  
  2. 状態の事後推定：イノベーション（観測残差）の計算には、非線形の観測方程式$\\boldsymbol{h}(\\cdot)$を用います。  
     $$ \\boldsymbol{\\hat{x}}\_k \= \\boldsymbol{\\hat{x}}\_k^- \+ \\boldsymbol{K}\_k (\\boldsymbol{z}\_k \- h(\\boldsymbol{\\hat{x}}\_k^-)) $$  
  3. 誤差共分散の事後推定：誤差共分散の更新式は、線形KFと同じ形式ですが、用いるカルマンゲイン$\\boldsymbol{K}\_kとヤコビ行列\\boldsymbol{H}\_k$が異なります。  
     Pk​=(I−Kk​Hk​)Pk−​

このように、EKFは状態の伝播そのものは非線形モデルで行い、不確かさ（共分散）の伝播は線形近似モデルで行うというハイブリッドなアプローチを取ります。

### **2.5. 近似の代償：誤差、発散、不安定性**

EKFの線形化はあくまで近似であり、この近似が性能上の限界と潜在的な問題点を生み出します。

* **線形化誤差**: システムの非線形性が強い場合、一次のテイラー展開は真の関数をうまく表現できず、大きな誤差を生む可能性があります 4。関数の「曲率」が推定値の周りで大きいほど、この近似誤差は深刻になります 12。  
* **フィルタの発散**: この線形化誤差が蓄積すると、フィルタが自身の不確かさを過小評価する（共分散行列$\\boldsymbol{P}$が不当に小さくなる）ことがあります。その結果、新しい観測値を無視するようになり、推定値が真値からどんどん乖離していく「発散」という現象を引き起こす可能性があります。  
* **非ガウス性の問題**: EKFは、非線形変換後の真の確率分布が非ガウス分布になるにもかかわらず、それを無理やりガウス分布（平均と共分散）で近似し続けます 17。この表現の不一致もまた、誤差の一因となります。

EKFは完璧な非線形フィルタではなく、むしろ実用的な妥協の産物です。その成功は、対象とする問題の非線形性の度合いに大きく依存します。しかし、その実装の容易さと計算効率の良さから、多くの応用分野で「最初に試されるべき非線形フィルタ」としての地位を確立しています 9。線形KFというよく知られた理論から非線形問題への移行が概念的に容易であり、組み込みシステムのような計算資源の限られた環境でもリアルタイム動作が可能であることが、その普及を後押ししてきました 10。EKFは、仕事をこなせる最もシンプルなツールを使うという、優れた工学原理の証左と言えるでしょう。

---

## **第3章 柔軟性の力：「モデル追加」という利点の解体**

ユーザーが提起した「後からモデルを追加できる」というEKFの利点は、単なる一機能ではなく、状態空間フィルタがナビゲーションやロボティクスの分野で支配的な地位を築いた、最も重要な実用的理由を指し示しています。この柔軟性は、EKFを単なる信号処理アルゴリズムから、真の情報融合エンジンへと昇華させます。この章では、この強力な特性を解体し、そのメカニズムと応用を具体的な事例を通して探求します。

### **3.1. 状態空間フレームワークのモジュール性**

この柔軟性の根源は、状態空間モデルが持つ本質的な「モジュール性」にあります 1。状態空間モデルは、システムの内部ダイナミクスを記述する「プロセスモデル」（EKFでは非線形関数$\\boldsymbol{f}(\\boldsymbol{x}, \\boldsymbol{u})

）と、状態と観測値の関係を記述する「観測モデル」（非線形関数\\boldsymbol{h}(\\boldsymbol{x})$）を明確に分離しています。

この分離のおかげで、一方のモデル（例えば観測モデル）を修正、拡張、あるいは完全に入れ替えても、もう一方のモデル（プロセスモデル）やフィルタの基本構造を変更する必要がありません。EKFはこのモジュール性を非線形システムへと継承し、実用的な設計の柔軟性を飛躍的に高めました。

### **3.2. 応用I \- センサーフュージョン：新たな情報源の統合**

「モデルの追加」という言葉の最も一般的な解釈は、新しい独立したセンサーを推定フレームワークに統合する「センサーフュージョン」です 18。

#### **3.2.1. メカニズム：観測ベクトルと観測モデルの拡張**

新しいセンサーを追加するプロセスは、驚くほど系統的です。

1. **観測ベクトルの拡張**：観測ベクトル$\\boldsymbol{z}\_k$を拡張し、新しいセンサーからの測定値をその成分として追加します。  
2. **観測モデルの拡張**：観測関数$\\boldsymbol{h}(\\boldsymbol{x}\_k)に、新しいセンサーの測定値が状態ベクトル\\boldsymbol{x}\_k$とどのように関連しているかをモデル化する新たな関数（行）を追加します。  
3. **観測ノイズ共分散行列の拡張**：観測ノイズ共分散行列$\\boldsymbol{R}$を拡張し、新しいセンサーのノイズ特性（分散）を対角成分に追加します。

この操作だけで、EKFは新しい情報源を自然に組み込むことができます。フィルタの核となる予測・更新アルゴリズム自体は変更されません。

#### **3.2.2. ケーススタディ：IMUとGPSの融合による高精度な車両自己位置推定**

この柔軟性の力を示す典型例が、慣性計測装置（IMU）と全地球測位システム（GPS）のセンサーフュージョンです 19。

* **初期モデル（IMUのみ）**：  
  * **状態ベクトル x**: 車両の位置、速度、姿勢（例：クォータニオン）など 22。  
  * **プロセスモデル f(⋅)**: IMUから得られる加速度と角速度（これらは制御入力$\\boldsymbol{u}$として扱われる）を積分し、状態を予測します。  
  * **課題**: このモデルは、短時間では高精度ですが、積分誤差が時間とともに蓄積し、推定位置が徐々に真の位置からずれていく「ドリフト」という現象に悩まされます 24。  
* **GPS「モデル」の追加**：  
  * GPSは、車両の絶対位置を直接、しかし比較的低い頻度（例：1Hz）で、かつノイズを含んだ形で提供します 25。  
  * **観測ベクトルの拡張**: 観測ベクトル$\\boldsymbol{z}\_k$にGPSの測位データを加えます。  
  * **観測モデルの拡張**: 観測関数$\\boldsymbol{h}(\\boldsymbol{x}\_k)に、状態ベクトル\\boldsymbol{x}\_k$から位置成分だけを抜き出す簡単な線形関数を追加します。  
  * **ヤコビ行列の更新**: 拡張された$\\boldsymbol{h}(\\cdot)に対応するヤコビ行列\\boldsymbol{H}\_k$を計算します。  
* 融合の結果：  
  EKFは、高周波で短期的な精度が高いIMUのデータと、低周波だが長期的なドリフトがないGPSのデータを、それぞれの信頼度（ノイズ共分散）に応じて最適に融合します。GPSのデータが利用可能なときは、IMUの蓄積誤差を補正します。GPSが利用できないトンネル内などでは、IMUのデータを用いて航法を継続します（推測航法）。これにより、各センサー単体の弱点を互いに補い合い、連続的で高精度かつロバストな自己位置推定が実現されます 19。

### **3.3. 応用II \- 状態ベクトル拡張：未知のシステムパラメータの推定**

「モデルの追加」という概念の、より高度で強力な応用が、システムの動的な状態だけでなく、センサーのバイアスやスケールファクタといった、静的あるいはゆっくり変動するパラメータ自体を推定対象に加える「状態ベクトル拡張」です。

#### **3.3.1. メカニズム：状態ベクトルの拡張**

この手法では、推定したい未知のパラメータを新たな状態変数とみなし、状態ベクトル$\\boldsymbol{x}\_k$に連結します。

1. **状態ベクトルの拡張**：元の状態ベクトルに、推定したいパラメータ（例：ジャイロバイアス$\\boldsymbol{b}*g$）を追加します。$\\boldsymbol{x}*{aug} \=^T$。  
2. プロセスモデルの拡張：追加したパラメータの時間発展をモデル化します。通常、バイアスのようなパラメータは一定、あるいはゆっくりと変動すると考えられるため、そのプロセスモデルは「ランダムウォーク」としてモデル化されます。  
   bg,k​=bg,k−1​+wbias​

   ここで$\\boldsymbol{w}\_{bias}$は、バイアスが時間とともにわずかに変動する可能性を表現するための小さなプロセスノイズです。  
3. 元のプロセスモデルも修正し、入力から現在のバイアス推定値を差し引くようにします。

#### **3.3.2. ケーススタディ：オンラインでのジャイロスコープバイアス推定**

* **問題**: ジャイロスコープには、時間や温度によって変動する「バイアス」と呼ばれる誤差が含まれており、これを補正しないと姿勢推定に深刻なドリフトが生じます 26。  
* **標準的なEKF**: 状態は姿勢（例：クォータニオン）のみ。ジャイロの測定値は入力$\\boldsymbol{u}$として扱われ、バイアスは事前に測定された固定値として差し引かれます。しかし、バイアスが変動すると誤差が生じます。  
* **拡張EKF**:  
  * **状態ベクトル**: 姿勢とジャイロバイアスを結合します。$\\boldsymbol{x} \= \[\\text{orientation}, \\text{gyro\_bias}\]^T$ 27。  
  * **プロセスモデル f(⋅)**: 2つの部分から構成されます。  
    1. 姿勢を更新する部分：ジャイロの測定値から、現在の**バイアス推定値**を引いた値を用いて姿勢を更新します。  
    2. バイアスを更新する部分：バイアスをランダムウォークとしてモデル化します。  
* 推定の仕組み:  
  加速度センサーや地磁気センサーなど、ジャイロとは別の手段で絶対的な姿勢（あるいはその一部）を観測できるとします 28。フィルタの更新ステップで、この外部情報に基づいて姿勢推定値が補正されると、その補正量（イノベーション）には、バイアスの誤差に関する情報が間接的に含まれています。EKFの数学的機構（特にカルマンゲインと共分散行列の更新）は、この情報を自動的にバイアスの状態変数へとフィードバックし、バイアス推定値をオンラインで継続的に修正します。

### **3.4. 統合：反復的設計のための柔軟なフレームワークとしてのEKF**

状態ベクトルと観測ベクトルをこのように拡張できる能力は、エンジニアがシステム開発において反復的なアプローチを取ることを可能にします 20。まず、核となる単純なモデルでフィルタを構築・テストし、その後、必要に応じて新たなセンサーやパラメータ推定器を段階的に追加していくことができます。これにより、複雑なシステムを一度に設計するのではなく、管理可能な単位で徐々に構築していくという、強力で実用的な開発ワークフローが実現します。

この柔軟性は、状態空間モデルが持つ「中央集権的」なアーキテクチャの直接的な帰結です。すべての情報（全センサーからの観測、ダイナミクスモデル）は、システムの状態に関する単一の中心的な「信念」（状態推定値$\\boldsymbol{\\hat{x}}と共分散行列\\boldsymbol{P}$）を参照して融合されます。新しい情報は、この中心的な信念に対する新たな制約として追加されるに過ぎません。このアーキテクチャは、個別のセンサーペアごとに場当たり的な補正を行う単純な相補フィルタなど 26 と比較して、はるかにスケーラブルで首尾一貫した情報融合を可能にします。したがって、EKFは単なるアルゴリズムではなく、複雑な推定システムを構築するための

**プラットフォーム**として機能するのです。

---

## **第4章 比較分析：EKFと他のオブザーバ**

EKFは非線形推定における重要なツールですが、唯一の選択肢ではありません。現代の推定理論の文脈でEKFを正しく位置づけるためには、その主要な代替手法との比較を通じて、それぞれの長所、短所、そして適用分野を明確にすることが不可欠です。この比較分析は、特定の問題に対して最適なフィルタを選択するための実践的な指針を提供します。

### **4.1. アンセンテッドカルマンフィルタ（UKF）：より優れたガウス近似**

EKFの根本的な弱点が、ヤコビ行列を用いた線形化近似にあることは既に述べました 4。アンセンテッドカルマンフィルタ（UKF）は、この問題を回避するために開発された、より洗練された手法です。

* **アンセンテッド変換（UT）**: UKFの核心は、非線形関数そのものを近似するのではなく、状態の**確率分布**をより巧みに近似する「アンセンテッド変換」にあります 29。UTは、現在の状態推定値の平均と共分散に基づき、決定論的に選ばれた少数のサンプル点（「シグマポイント」と呼ばれる）を生成します。  
* **UKFのアルゴリズム**:  
  1. 現在の状態推定値の周りに、シグマポイントを生成します。  
  2. これらのシグマポイントを、個別に、**真の非線形関数**（$\\boldsymbol{f}$または$\\boldsymbol{h}$）に直接通して変換します。解析的な線形化は一切行いません。  
  3. 変換されたシグマポイントの重み付き平均と共分散を計算することで、変換後の分布の平均と共分散を再構成します。  
* **EKFとUKFの比較**:  
  * **精度**: UKFは、特に非線形性の強いシステムにおいて、EKFよりも一般的に高精度です。UTは、テイラー展開の少なくとも二次モーメントまでを正確に捉えることができるのに対し、EKFの線形化は一次までしか考慮しません 28。  
  * **実装**: UKFは、開発者がモデルのヤコビ行列を解析的に導出する必要がないという大きな利点があります 31。これは、モデルが複雑な場合に開発の手間を大幅に削減します。ただし、アルゴリズム自体の概念はEKFより複雑です。  
  * **計算コスト**: 状態ベクトルの次元数に依存しますが、計算コストはEKFと同等か、わずかに高くなることが多いです 33。ヤコビ行列の計算コストは不要になりますが、複数のシグマポイントを伝播させるコストが加わります 31。

UKFは、EKFの線形化誤差が問題となるが、依然としてガウス分布の仮定が妥当であるような状況において、非常に強力な代替案となります。

### **4.2. パーティクルフィルタ（PF）：ガウス分布の仮定を超えて**

EKFとUKFは共に、システムの信念（状態の確率分布）が単峰性であり、ガウス分布で十分に近似できることを前提としています。しかし、この仮定が成り立たない問題も数多く存在します。例えば、ロボットが完全に道に迷った場合（自己位置を見失った場合）、その存在確率は複数の離れた場所にピークを持つ多峰性の分布になるかもしれません。このような状況に対処するのがパーティクルフィルタ（PF）です。

* **モンテカルロ法に基づくアプローチ**: PFは、確率分布を解析的な式（平均と共分散）で表現する代わりに、多数のランダムなサンプル（「パーティクル」と呼ばれる）の集合とその重みで近似します 34。パーティクルの密度が高い領域が、確率の高い領域に対応します。  
* **PFのアルゴリズム（逐次重要度再サンプリング法 \- SIR）**:  
  1. **予測**: 各パーティクルを、プロセスモデル$\\boldsymbol{f}(\\cdot)$に従って移動させ、さらにランダムなノイズを加えます 37。  
  2. **更新**: 新しい観測値$\\boldsymbol{z}\_kが得られると、各パーティクルの状態がその観測値をどれだけうまく説明できるか（尤度\\boldsymbol{p}(\\boldsymbol{z}\_k|\\boldsymbol{x}\_k)$）に基づいて、各パーティクルの「重み」を計算します 37。  
  3. **再サンプリング（リサンプリング）**: 重みに比例した確率で、現在のパーティクル集合から新しいパーティクル集合を（重複を許して）再サンプリングします。これにより、重みの大きいパーティクルは複製され、重みの小さいパーティクルは消滅します 36。  
* **EKFとPFの比較**:  
  * **柔軟性**: PFは最も柔軟なフィルタです。任意の確率分布を表現でき 22、微分不可能なモデルや非ガウスノイズにも対応できます。システムモデルは、そこからサンプリングさえできれば、どのようなものでも構いません 34。  
  * **精度**: PFが対象とするような問題（強い非線形性、非ガウス性、多峰性）においては、EKFやUKFよりもはるかに高い精度を達成できます 32。  
  * **計算コスト**: PFは計算コストが非常に高いという大きな欠点があります。その性能はパーティクルの数に直接依存し、しばしば数千から数万以上のパーティクルが必要となります 22。これは高次元の状態空間では「次元の呪い」によりさらに深刻になります。  
  * **実装**: 概念は比較的単純ですが、パーティクルの枯渇（少数のパーティクルに重みが集中し多様性が失われる問題）を避けるための工夫など、実践的な調整が難しい場合があります。

### **4.3. 制御理論の広範な文脈におけるEKF**

EKFのような確率的なフィルタは、制御理論における決定論的な非線形オブザーバ（例：高ゲインオブザーバ、スライディングモードオブザーバ）と比較されることもあります。両者の根本的な違いは、カルマンフィルタファミリーが本質的に**確率的**である点にあります。これらは、不確かさを陽にモデル化し、共分散行列$\\boldsymbol{P}$として伝播させます。一方、決定論的なオブザーバは、通常、不確かさの尺度を提供せず、特定の条件下での推定誤差の収束保証に焦点を当てます 40。

### **表4.1: 状態推定フィルタの比較概要**

| 特徴 | 線形カルマンフィルタ (KF) | 拡張カルマンフィルタ (EKF) | アンセンテッドカルマンフィルタ (UKF) | パーティクルフィルタ (PF) |
| :---- | :---- | :---- | :---- | :---- |
| **システムモデル** | 線形 | 微分可能な非線形 | 任意の非線形 | 任意の非線形（サンプリング可能） |
| **ノイズの仮定** | ガウス分布 | ガウス分布 | ガウス分布 | 任意 |
| **中核メカニズム** | 直接的な行列演算 | ヤコビ行列による線形化 | アンセンテッド変換 | 逐次モンテカルロ法（重点的再サンプリング） |
| **計算コスト** | 低 | 低～中 | 中 | 高～非常に高い |
| **主な利点** | 線形システムで最適かつ高速 | 実装の相対的容易さと速度 | 高精度かつヤコビ行列不要 | 任意の分布を表現可能 |
| **主な限界** | 線形性の制約 | 線形化誤差と発散リスク | ガウス分布の制約と計算コスト | 高い計算コストと次元の呪い |

この表は、KFからPFへの進化が、**より大きな表現力と非線形性・非ガウス性への頑健性を得るために、計算の複雑性を増大させるという明確なトレードオフの軌跡**を描いていることを示しています。これらは無関係なアルゴリズムの寄せ集めではなく、元のKFの厳格な仮定を段階的に緩和していく論理的な発展系列なのです。どのフィルタを選択するかは、問題の「次元の呪い」と「複雑性の呪い」のバランスを取ることに他なりません。EKFは高次元でも非線形性が穏やかならばうまく機能します。PFは低次元で複雑な分布を扱うのに優れていますが、高次元ではパーティクルが疎になりすぎて性能が劣化します。UKFは、EKFよりも非線形性をうまく扱い、かつPFのような高次元での指数的な複雑性の問題もないため、両者の中間に位置する魅力的な選択肢となります。

---

## **第5章 実践的実装と結論**

理論的な理解を深めた上で、EKFを現実の問題に適用する際には、いくつかの実践的な考慮事項が重要となります。フィルタの性能は、理論的な正しさだけでなく、適切なチューニングと潜在的な落とし穴への対処に大きく依存します。

### **5.1. チューニングの技術：共分散行列QとR**

EKFの性能を左右する最も重要なパラメータが、プロセスノイズ共分散行列$\\boldsymbol{Q}と観測ノイズ共分散行列\\boldsymbol{R}$です 10。これらは、フィルタにシステムモデルと観測値のどちらをどれだけ信用させるかを指示する「チューニングノブ」として機能します。

* **観測ノイズ共分散 R**: この値は、比較的決定しやすいことが多いです。センサーのデータシートに記載されている精度情報や、静止状態でセンサーデータを収集し、その分散を計算するなどの経験的なテストから設定できます。  
* **プロセスノイズ共分散 Q**: $\\boldsymbol{Q}$の決定はより難しく、しばしば「技術」と見なされます。これは、プロセスモデル（状態方程式）が現実のダイナミクスをどれだけ正確に捉えられているかという不確かさを表します。  
  * **$\\boldsymbol{Q}$を大きくする**: フィルタはプロセスモデルをあまり信用しなくなり、観測値に対してより敏感に反応するようになります。これにより、推定値の応答性は高まりますが、ノイズが多く含まれる可能性があります。  
  * **$\\boldsymbol{Q}$を小さくする**: フィルタはプロセスモデルを強く信用するようになり、観測ノイズの影響を平滑化して、より滑らかな推定値を生成します。しかし、モデル化されていないダイナミクスの変化に追従するのが遅れる可能性があります 1。

実践的には、$\\boldsymbol{R}をまず設定し、その後、シミュレーションや実機での試行錯誤を通じて、望ましい性能が得られるように\\boldsymbol{Q}$を反復的に調整していくのが一般的なアプローチです。

### **5.2. よくある落とし穴とその回避策**

* **フィルタの発散**: 前述の通り、線形化誤差や不適切な$\\boldsymbol{Q}、\\boldsymbol{R}$の値が原因で、推定値が真値から乖離していく現象です。発散が疑われる場合は、以下の対策が考えられます。  
  1. $\\boldsymbol{Q}$の値を大きくして、モデルへの依存度を下げる。  
  2. より高精度なUKFや、問題の性質によってはPFへの切り替えを検討する。  
  3. イノベーション（観測残差）がその共分散に対して統計的に整合性が取れているかをチェックする（カイ二乗検定など）ことで、フィルタの「健全性」を監視する。  
* **数値的不安定性**: 共分散行列$\\boldsymbol{P}は、その定義上、常に対称かつ半正定値でなければなりません。しかし、コンピュータの浮動小数点演算に起因する丸め誤差などにより、この性質が崩れ、フィルタが不安定になることがあります。この問題に対処するため、\\boldsymbol{P}$の代わりにその平方根を伝播させる、より数値的に安定した「平方根カルマンフィルタ」などの派生形が用いられることもあります。

### **5.3. 意思決定フレームワーク：適切なフィルタの選択**

どのフィルタを使用すべきかという問いに対して、以下のような意思決定の指針が役立ちます。

1. **対象システムは線形で、ノイズはガウス分布と仮定できますか？**  
   * はい → \*\*線形カルマンフィルタ（KF）\*\*を使用してください。これは最も効率的で最適な選択です。  
2. **システムは非線形ですか？**  
   * はい →  
     * **非線形性は穏やかで、ヤコビ行列の導出は容易ですか？**  
       * はい → まずは\*\*拡張カルマンフィルタ（EKF）\*\*から試してください。多くの場合、十分な性能を低コストで得られます。  
     * **非線形性が強い、またはヤコビ行列の導出が困難または非効率ですか？**  
       * はい → \*\*アンセンテッドカルマンフィルタ（UKF）\*\*を検討してください。精度向上と実装の簡素化が期待できます。  
     * **ノイズが明らかに非ガウス分布である、または確率分布が多峰性（複数の仮説が同時に存在する）を持つ可能性がありますか？**  
       * はい → \*\*パーティクルフィルタ（PF）\*\*が必要です。EKFやUKFでは原理的に対処できません。

このフレームワークは、問題の特性に基づいて、最も適切なフィルタリング手法を選択するための実践的な出発点を提供します。

### **5.4. 結論：EKFの不朽の遺産**

拡張カルマンフィルタは、その理論的な限界や、より強力な代替手法が存在するにもかかわらず、現代の推定理論と応用の分野において、依然として揺るぎない地位を占めています 9。

EKFが提供する、妥当な性能、高い計算効率、そして相対的な実装の容易さという組み合わせは、エンジニアの道具箱の中で非常に価値のあるツールであり続けています。それは、エレガントな線形理論と、複雑で厄介な非線形の現実との間に架けられた、極めて重要な橋渡しです。

さらに重要なのは、EKFが体現する原理、すなわち確率的な状態推定、複数情報源の体系的な融合、そして反復的なモデル拡張という考え方です。これらの原理は、自動運転車、ドローン、ロボティクス、金融工学といった多様な分野で、かつてないほど重要性を増しています。EKFは、単なるアルゴリズムとしてだけでなく、我々が動的な世界の不確かさに立ち向かうための、強力な思考の枠組みとして、その遺産を残し続けるでしょう。

#### **引用文献**

1. カルマンフィルタの考え方 \- Logics of Blue, 6月 20, 2025にアクセス、 [https://logics-of-blue.com/kalman-filter-concept/](https://logics-of-blue.com/kalman-filter-concept/)  
2. Pythonによるビジネス予測に活かす「状態空間モデル」の基礎と実装例 \- セールスアナリティクス, 6月 20, 2025にアクセス、 [https://www.salesanalytics.co.jp/datascience/datascience250/](https://www.salesanalytics.co.jp/datascience/datascience250/)  
3. カルマンフィルター (Kalman filter) を試す | 株式会社エムケイシステム TECH BLOG, 6月 20, 2025にアクセス、 [https://blog.mksc.jp/contents/kalman\_filter/](https://blog.mksc.jp/contents/kalman_filter/)  
4. カルマンフィルタの導出 \- 何時もの話っ！, 6月 20, 2025にアクセス、 [https://memo.soarcloud.com/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%81%AE%E5%B0%8E%E5%87%BA/](https://memo.soarcloud.com/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%81%AE%E5%B0%8E%E5%87%BA/)  
5. カルマンフィルタ \- ごちきか, 6月 20, 2025にアクセス、 [https://gochikika.ntt.com/Modeling/kalman\_principle.html](https://gochikika.ntt.com/Modeling/kalman_principle.html)  
6. 素人によるカルマンフィルタの基礎の入門 \#制御工学 \- Qiita, 6月 20, 2025にアクセス、 [https://qiita.com/sakaeda11/items/6b9bfa2e922304b5edab](https://qiita.com/sakaeda11/items/6b9bfa2e922304b5edab)  
7. 拡張カルマンフィルタによる自己位置推定動作の可視化 \#EKF \- Qiita, 6月 20, 2025にアクセス、 [https://qiita.com/Crafty\_as\_a\_Fox/items/55448e2ed9ce0f340814](https://qiita.com/Crafty_as_a_Fox/items/55448e2ed9ce0f340814)  
8. 非線形システムに対する状態推定フィルタの設計, 6月 20, 2025にアクセス、 [https://www.topic.ad.jp/sice/htdocs/papers/335/335-2.pdf](https://www.topic.ad.jp/sice/htdocs/papers/335/335-2.pdf)  
9. 【数分解説】拡張カルマンフィルタ : 非線形でもノイズを考慮してリアルタイムに直接観測できない状態を推定したい【Extended Kalman FIlter】 \- YouTube, 6月 20, 2025にアクセス、 [https://www.youtube.com/watch?v=Yd6sn0f5BKI](https://www.youtube.com/watch?v=Yd6sn0f5BKI)  
10. 線形カルマンフィルタの基礎を理解するための基本概念と構成要素, 6月 20, 2025にアクセス、 [https://www.issoh.co.jp/tech/details/6059/](https://www.issoh.co.jp/tech/details/6059/)  
11. カルマンフィルター：現代技術における不可欠な予測と補正のアルゴリズム | Reinforz Insight, 6月 20, 2025にアクセス、 [https://reinforz.co.jp/bizmedia/25980/](https://reinforz.co.jp/bizmedia/25980/)  
12. 車体横すべり角 を推定する線形オブザーバに関する考察 \- 東京大学, 6月 20, 2025にアクセス、 [http://hflab.k.u-tokyo.ac.jp/hori\_lab/paper\_2003/data/tomoko/sice.pdf](http://hflab.k.u-tokyo.ac.jp/hori_lab/paper_2003/data/tomoko/sice.pdf)  
13. 拡張カルマンフィルター（EKF）の説明｜Ultralytics ウルトラリティクス, 6月 20, 2025にアクセス、 [https://www.ultralytics.com/ja/glossary/extended-kalman-filter-ekf](https://www.ultralytics.com/ja/glossary/extended-kalman-filter-ekf)  
14. カルマンフィルター \- Wikipedia, 6月 20, 2025にアクセス、 [https://ja.wikipedia.org/wiki/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%83%BC](https://ja.wikipedia.org/wiki/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%83%BC)  
15. 第 11 章 拡張カルマンフィルタ \- Python で学ぶベイズフィルタとカルマンフィルタ (翻訳), 6月 20, 2025にアクセス、 [https://inzkyk.xyz/kalman\_filter/extended\_kalman\_filters/](https://inzkyk.xyz/kalman_filter/extended_kalman_filters/)  
16. 博士学位論文 等価入力外乱手法を用いた 非線形特性の推定・補償に関する研究 \- 東京工科大学, 6月 20, 2025にアクセス、 [https://www.teu.ac.jp/ap\_page/koukai/H29\_02\_3\_isiki.pdf](https://www.teu.ac.jp/ap_page/koukai/H29_02_3_isiki.pdf)  
17. 拡張カルマンフィルタ, 6月 20, 2025にアクセス、 [https://sterngerlach.github.io/doc/extended-kalman-filter.pdf](https://sterngerlach.github.io/doc/extended-kalman-filter.pdf)  
18. IMUを活用してロボットの位置推定機能を強化、より高精度なナビゲーションが可能に, 6月 20, 2025にアクセス、 [https://www.analog.com/jp/resources/analog-dialogue/articles/enhancing-robotic-localization.html](https://www.analog.com/jp/resources/analog-dialogue/articles/enhancing-robotic-localization.html)  
19. IMUで補完！GNSS電波圏外で正しく測位する技術 \- ZEPエンジニアリング, 6月 20, 2025にアクセス、 [https://www.zep.co.jp/before\_after\_pcb/article/zmag\_robot\_2024-da3/](https://www.zep.co.jp/before_after_pcb/article/zmag_robot_2024-da3/)  
20. Self Driving and ROS 2 \- Learn by Doing\! Odometry & Control: カルマンフィルタ (セクション11-2/13)｜Hafnium \- note, 6月 20, 2025にアクセス、 [https://note.com/hafnium/n/n90a7610b2210](https://note.com/hafnium/n/n90a7610b2210)  
21. GPS とセンサの組み合わせによる自己位置推定システムの開発 （第３報） \- 福島県, 6月 20, 2025にアクセス、 [https://www.pref.fukushima.lg.jp/uploaded/life/678059\_1907272\_misc.pdf](https://www.pref.fukushima.lg.jp/uploaded/life/678059_1907272_misc.pdf)  
22. 拡張カルマンフィルタ(EKF)を速習！数式の導出と可視化で学ぶ \- Qiita, 6月 20, 2025にアクセス、 [https://qiita.com/scomup/items/a23938a9d9a17fafa3f2](https://qiita.com/scomup/items/a23938a9d9a17fafa3f2)  
23. クォータニオンを用いたカルマンフィルタによるGPS/IMUを複合 ..., 6月 20, 2025にアクセス、 [https://qiita.com/rsasaki0109/items/a5b13c13a3bb76afb7a1](https://qiita.com/rsasaki0109/items/a5b13c13a3bb76afb7a1)  
24. Turing Tech Talk 第17回 「次世代自動運転を支える自己位置推定 多センサー融合による精度と信頼性の追求」, 6月 20, 2025にアクセス、 [https://tur.ing/turipo/CX1h1VsM](https://tur.ing/turipo/CX1h1VsM)  
25. カルマンフィルタを用いたセンサフュージョン(1) \- Weekly Engineering, 6月 20, 2025にアクセス、 [https://weeklyengineering.com/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%82%92%E7%94%A8%E3%81%84%E3%81%9F%E3%82%BB%E3%83%B3%E3%82%B5%E3%83%95%E3%83%A5%E3%83%BC%E3%82%B8%E3%83%A7%E3%83%B31/](https://weeklyengineering.com/%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%82%92%E7%94%A8%E3%81%84%E3%81%9F%E3%82%BB%E3%83%B3%E3%82%B5%E3%83%95%E3%83%A5%E3%83%BC%E3%82%B8%E3%83%A7%E3%83%B31/)  
26. 備忘録：相補フィルタとカルマンフィルタ？｜しろっぺ！ \- note, 6月 20, 2025にアクセス、 [https://note.com/ruber\_se\_kr/n/n2ff21b5a009c](https://note.com/ruber_se_kr/n/n2ff21b5a009c)  
27. パラメータ推定（２）カルマンフィルター \#制御工学 \- Qiita, 6月 20, 2025にアクセス、 [https://qiita.com/taka\_horibe/items/f85553c0ef842658e427](https://qiita.com/taka_horibe/items/f85553c0ef842658e427)  
28. 6軸IMU慣性センサ～拡張カルマンフィルタ \- 何時もの話っ！, 6月 20, 2025にアクセス、 [https://memo.soarcloud.com/6%E8%BB%B8imu%EF%BD%9E%E6%8B%A1%E5%BC%B5%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF/](https://memo.soarcloud.com/6%E8%BB%B8imu%EF%BD%9E%E6%8B%A1%E5%BC%B5%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF/)  
29. 無香料カルマンフィルタとは \- 詳細な概要 \- 統計を簡単に学ぶ, 6月 20, 2025にアクセス、 [https://ja.statisticseasily.com/%E7%94%A8%E8%AA%9E%E9%9B%86/%E7%84%A1%E9%A6%99%E6%96%99%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%81%AE%E8%A9%B3%E7%B4%B0%E3%81%AA%E6%A6%82%E8%A6%81%E3%81%A8%E3%81%AF/](https://ja.statisticseasily.com/%E7%94%A8%E8%AA%9E%E9%9B%86/%E7%84%A1%E9%A6%99%E6%96%99%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%81%AE%E8%A9%B3%E7%B4%B0%E3%81%AA%E6%A6%82%E8%A6%81%E3%81%A8%E3%81%AF/)  
30. パーティクルフィルタによる計測誤差の低減に関す る研究 \- CORE, 6月 20, 2025にアクセス、 [https://core.ac.uk/download/477994157.pdf](https://core.ac.uk/download/477994157.pdf)  
31. 扩展卡尔曼滤波（EKF）和无迹卡尔曼滤波（UKF）的比较原创 \- CSDN博客, 6月 20, 2025にアクセス、 [https://blog.csdn.net/DX\_LI/article/details/139175052](https://blog.csdn.net/DX_LI/article/details/139175052)  
32. SLAM～Unscentedカルマンフィルタ \- 何時もの話っ！, 6月 20, 2025にアクセス、 [https://memo.soarcloud.com/slam%EF%BD%9Eunscented%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF/](https://memo.soarcloud.com/slam%EF%BD%9Eunscented%E3%82%AB%E3%83%AB%E3%83%9E%E3%83%B3%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF/)  
33. パラメータ推定のためのデュアルフィルタの設計と 動的 ... \- 土木学会, 6月 20, 2025にアクセス、 [http://library.jsce.or.jp/jsce/open/00057/2012/68-D-0031.pdf](http://library.jsce.or.jp/jsce/open/00057/2012/68-D-0031.pdf)  
34. 【数分解説】パーティクルフィルタ(粒子フィルタ): 観測できない ..., 6月 20, 2025にアクセス、 [https://m.youtube.com/watch?v=ICjgR6h7vDg\&pp=ygUQI-eKtuaFi-aOqOWumuWZqA%3D%3D](https://m.youtube.com/watch?v=ICjgR6h7vDg&pp=ygUQI-eKtuaFi-aOqOWumuWZqA%3D%3D)  
35. 粒子フィルタについて解説 \- AGIRobots Blog, 6月 20, 2025にアクセス、 [https://developers.agirobots.com/jp/particle-filter/](https://developers.agirobots.com/jp/particle-filter/)  
36. パーティクルフィルタ, 6月 20, 2025にアクセス、 [https://sterngerlach.github.io/doc/particle-filter.pdf](https://sterngerlach.github.io/doc/particle-filter.pdf)  
37. 第 12 章 粒子フィルタ \- Python で学ぶベイズフィルタと ... \- inzkyk.xyz, 6月 20, 2025にアクセス、 [https://inzkyk.xyz/kalman\_filter/particle\_filters/](https://inzkyk.xyz/kalman_filter/particle_filters/)  
38. コンピュータビジョンのセカイ \- 今そこにあるミライ(23) パーティクルフィルタによる観測技術, 6月 20, 2025にアクセス、 [https://news.mynavi.jp/techplus/article/computer\_vision-23/](https://news.mynavi.jp/techplus/article/computer_vision-23/)  
39. B4-8 パーティクルフィルタを用いた自律移動ロボットの行動予測制御, 6月 20, 2025にアクセス、 [https://www.spice.ci.ritsumei.ac.jp/\~ymaeda/article/h20/c40.pdf](https://www.spice.ci.ritsumei.ac.jp/~ymaeda/article/h20/c40.pdf)  
40. 極配置によるオブザーバ型 自動抽出制御の設計について \- 鹿児島大学リポジトリ, 6月 20, 2025にアクセス、 [https://ir.kagoshima-u.ac.jp/record/1996/files/AN00040363\_v51\_p1-6.pdf](https://ir.kagoshima-u.ac.jp/record/1996/files/AN00040363_v51_p1-6.pdf)