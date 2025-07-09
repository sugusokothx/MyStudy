

# **IPMモータ磁束推定のための拡張カルマンフィルタに関する包括的技術レビュー**

## **1.0 調査結果のエグゼクティブサマリー**

### **総合評価**

提示されたMATLABコードは、挑戦的な推定問題に対する洗練された、かつ大部分において適切に構造化された実装です。2次モータモデルを採用し、解析的なヤコビ行列のためにヘシアン行列を利用している点は、高度な理論的理解を示しています。状態ダイナミクスおよびヤコビ行列Fの核心的な数学的導出は、アルゴリズムとして健全であることが確認されました。

### **重要な発見事項**

開発者の意図と実装の間に重大な不一致が特定されました。共分散更新ステップのコメントには「安定性のためのJoseph形式」と記載されていますが、実際のコードはより単純な標準更新式を実装しています。これは、フィルタの数値的堅牢性を損ない、特定の動作条件下で発散を引き起こす可能性のある**致命的なバグ**です。

### **主な強み**

* 非線形IPMモータに対する状態空間モデルの正しい定式化。  
* 複雑な4x4状態遷移行列Fの正確かつ成功した解析的導出。  
* 状態管理のための永続変数の適切な使用による、クリーンなコード構造。

### **主要な推奨事項**

1. **即時修正:** 数値的安定性を確保するため、共分散更新式を直ちに真のJoseph形式に修正すること。  
2. **再調整:** プロセスノイズ共分散行列Q、特に磁束偏差項（Q\_phi）のチューニングを見直し、物理的な温度変化率を現実的に反映させること。  
3. **検討:** 前進オイラー法による離散化のトレードオフを考慮すること。ただし、指定された5kHzのサンプリングレートでは十分である可能性が高い。

---

## **2.0 IPMモータ状態空間モデルの分析**

本セクションでは、IPMモータの物理モデルを解剖し、それがEKFのための状態空間表現にどのように変換されたかを分析することで、理論的基礎を築きます。

### **2.1 非線形IPMモータの連続時間d-q軸モデル**

#### **基本方程式**

全ての分析の基礎となるのは、回転子同期（d-q）座標系における標準的な連続時間電圧方程式です。これらの方程式は、固定子抵抗、磁束鎖交の時間変化率、および逆起電力（回転電圧）項を考慮に入れています 1。

vd​=Rs​id​+dtdϕd​​−ωϕq​vq​=Rs​iq​+dtdϕq​​+ωϕd​  
提供されたコードは、これらの式を$\\frac{d\\phi}{dt}$について解くために正しく再整理しています。

#### **非線形性と磁束鎖交**

150kW級の高出力IPMモータでは、磁気飽和により磁束鎖交（ϕd​,ϕq​）は電流（id​,iq​）の強い非線形関数となります。このモデルは、総磁束をベースラインとなるマップ由来の磁束（ϕd0​,ϕq0​）と推定された偏差（dϕd​,dϕq​）の和として正しく表現しています。

ϕd​=ϕd0​(id​,iq​)+dϕd​ϕq​=ϕq0​(id​,iq​)+dϕq​  
ベースライン磁束にルックアップテーブル（マップ）を使用するこのアプローチは、非線形機をモデル化するための標準的な産業慣行です 1。EKFの役割は、このベースラインからの偏差を推定することにあり、これは非常に強力な手法です。

### **2.2 状態ベクトルとシステム入力**

* **状態ベクトル x:** 状態ベクトルとして x=T を選択したのは論理的です。これには直接観測される状態（電流）と、推定対象の観測されない状態（磁束偏差）が含まれています。  
* **入力ベクトル u:** 入力ベクトル u=\[vd​,vq​,ω\]T は、システムダイナミクスを駆動する制御入力（電圧）と主要な変動パラメータ（角速度）を正しく特定しています。

#### **固定子抵抗 Rs の役割に関する考察**

このモデルでは、固定子抵抗Rsが定数（30e-3 Ω）として定義されています。しかし、磁束変化（Δϕ）を推定する主な動機は**温度変化**です。固定子抵抗もまた温度に強く依存し、動作範囲全体で40%以上変動することも珍しくありません。

EKFモデルは、この固定Rs値を用いて電流を予測します。もし実際のRsが温度によって変化すれば、モデル化誤差が生じます。EKFはこの誤差を補償しようと試みますが、電圧方程式において磁束項と抵抗項は結合しているため、フィルタはRsの変化に起因する電圧誤差を、誤ってΔφの変化として解釈してしまう可能性があります。これはパラメータのクロスカップリングまたは可観測性の問題として知られています。

このことから、推定されたΔφがモデル化されていないRsの変化によって「汚染」される重大なリスクが存在します。したがって、以下のいずれかの対策を強く推奨します。

a) 状態ベクトルを拡張し、Rsを5番目の状態としてオンラインで推定する。これは先進的なモータ制御で一般的に用いられる強力な手法です 3。

b) 別途、より単純な温度推定器（例：熱モデルに基づく）を実装し、適応させたRs値を各タイムステップでEKFに供給する。  
この考察は、提供されたコードを超えて、EVアプリケーションにおける根本的なモデリングの仮定とその実践的な結果に疑問を投げかけるものです。

### **2.3 状態予測関数 (f\_discrete\_delta\_phi) と離散化**

#### **電流ダイナミクスの導出**

この関数は、電圧方程式から磁束の時間微分phi\_dotを正しく計算しています。次に、関係式 dtdi​=J−1dtdϕ​ を用いて電流ダイナミクスidiq\_dotを導出しており、これは有効かつ洗練された定式化です。

#### **離散化手法**

コードは離散化のために1次の前進オイラー法（x\_next \= x \+ Ts \* x\_dot）を使用しています。これは最も単純な離散化手法です 7。サンプリング時間

Tsが200µs（5kHz）と非常に高速であることを考えると、この選択は妥当です。電気的ダイナミクスは高速ですが、この時間スケールでは1ステップあたりの変化は小さくなります。前進オイラー法の主な欠点は、システムのダイナミクスに対して時間ステップが大きい場合に不正確さや不安定性を引き起こすことですが、この高いサンプリングレートでは、オイラー法によって生じる誤差は、フィルタが除去するように設計されているノイズレベル内に収まる可能性が高いです。ルンゲ・クッタ法のような高次の手法はより高い精度を提供しますが、リアルタイム組込みシステムにとって大きな懸念事項である計算コストが大幅に増加します 8。したがって、オイラー法の選択は、計算効率を優先する現実的なエンジニアリング判断と言えます。

#### **磁束偏差モデル (Δφ)**

モデルはΔφがランダムウォークに従うと仮定しており、その予測値は前の値と同じ（dphid\_next \= dphid）になります。これは、ダイナミクスが明示的に分かっていないゆっくりと変化するパラメータ（温度に起因する磁束変化など）をモデル化するための標準的かつ適切な方法です。そして、Q行列がその1ステップあたりの変化の期待される大きさを決定します。

### **表 2.1: EKFシステムモデルの定義**

| 要素 | 記号 | 定義 | 単位 |
| :---- | :---- | :---- | :---- |
| 状態ベクトル | x | T | A, A, Wb, Wb |
| 入力ベクトル | u | \[vd​,vq​,ω\]T | V, V, rad/s |
| 観測ベクトル | z | \[id\_meas​,iq\_meas​\]T | A, A |
| 状態遷移行列 | F | ∂x∂f​ (4x4行列) | \- |
| 観測行列 | H | ∂x∂h​ (2x4行列) | \- |
| プロセスノイズ共分散 | Q | (4x4行列) | A², A², Wb², Wb² |
| 観測ノイズ共分散 | R | (2x2行列) | A², A² |

---

## **3.0 解析的状態遷移行列（F）の検証**

このセクションは、本実装で最も数学的に複雑な部分である、解析的なヤコビ行列計算を綿密に検証します。

### **3.1 連続時間ヤコビ行列 A の第一原理からの導出**

まず、連続時間システム行列 A=∂x∂f​ の完全な導出を段階的に示します。ここで、f は連続時間状態微分のベクトル T です。

電流ダイナミクスの微分（∂x∂(di/dt)​）が最も複雑な部分です。  
dtdi​=w=J(i)−1⋅ϕdot​(i,Δϕ) の関係から、積の微分法則を用いると、  
∂xj​∂w​=∂xj​∂(J−1)​ϕdot​+J−1∂xj​∂(ϕdot​)​  
行列の逆行列の微分に関する恒等式 ∂xj​∂(J−1)​=−J−1∂xj​∂J​J−1 を用いて代入すると、

∂xj​∂w​=−J−1∂xj​∂J​J−1ϕdot​+J−1∂xj​∂(ϕdot​)​  
これは、w \= J\_inv\*phi\_dot および DP が ∂x∂(ϕdot​)​ を表す、ユーザーのコードで実装されている式と完全に一致します。

### **3.2 実装されたヤコビ行列 calculate\_F\_delta\_phi\_analytic の分析**

コードを正式な導出と一行ずつ比較します。

* **dJ/dx 項:** コードはヘシアン行列 Hd と Hq を用いて did​dJ​ と diq​dJ​ を正しく表現しています。また、インダクタンスマップ J は電流のみの関数であるため、d(Δϕ)dJ​ はゼロであると正しく仮定しています。これは2次モデルの重要な特徴です。  
* **DP 行列 (∂x∂(ϕdot​)​):** DP行列の8つの非ゼロ要素それぞれの導出を示し、-Rs \+ omega\*Lqd や \-omega\*Ldd といった項が正しいことを確認します。これにより、クロスカップリング効果（Ldq, Lqd）と逆起電力の依存性が適切に微分されていることが確認できます。

### **3.3 ヤコビ行列の離散化**

連続時間ヤコビ行列Aと離散時間状態遷移行列Fの関係は F=eATs​ です。小さいTsに対しては、1次のテイラー展開で近似できます: F≈I+ATs​。ユーザーのコードはこれを F(1,idx) \= 1 \+ Ts\*dwdx(1) のように直接実装しており、状態予測における前進オイラー法の選択と一致しています。

### **3.4 ヤコビ行列の正当性に関する結論**

最終的な判断として、解析的ヤコビ行列Fのユーザーによる実装は、**定義された状態モデルと数学的に一致しており、正しい**と結論付けます。これは、関連する複雑さを考慮すると、重要な達成事項です。このコード部分は、本実装の主要な強みとして評価されます。

### **表 3.1: ヤコビ行列要素の比較分析**

| 要素 | 導出された理論式 | コード実装 | 状態 |
| :---- | :---- | :---- | :---- |
| F(1,1) | 1+Ts​⋅(∂id​∂(did​/dt)​) | F(1,1) \= 1 \+ Ts\*dwdx(1); (idx=1) | ✓ |
| F(1,2) | Ts​⋅(∂iq​∂(did​/dt)​) | F(1,2) \= Ts\*dwdx(1); (idx=2) | ✓ |
| F(1,3) | Ts​⋅(∂Δϕd​∂(did​/dt)​) | F(1,3) \= Ts\*dwdx(1); (idx=3) | ✓ |
| F(1,4) | Ts​⋅(∂Δϕq​∂(did​/dt)​) | F(1,4) \= Ts\*dwdx(1); (idx=4) | ✓ |
| F(2,1) | Ts​⋅(∂id​∂(diq​/dt)​) | F(2,1) \= Ts\*dwdx(2); (idx=1) | ✓ |
| F(2,2) | 1+Ts​⋅(∂iq​∂(diq​/dt)​) | F(2,2) \= 1 \+ Ts\*dwdx(2); (idx=2) | ✓ |
| F(2,3) | Ts​⋅(∂Δϕd​∂(diq​/dt)​) | F(2,3) \= Ts\*dwdx(2); (idx=3) | ✓ |
| F(2,4) | Ts​⋅(∂Δϕq​∂(diq​/dt)​) | F(2,4) \= Ts\*dwdx(2); (idx=4) | ✓ |
| F(3,3) | 1 | eye(4)による初期化 | ✓ |
| F(4,4) | 1 | eye(4)による初期化 | ✓ |
| *注: dwdxはループ内で各状態変数について計算されます。上表のF(i,j)のコード実装は、対応するidxループ時のdwdxの計算結果を反映しています。* |  |  |  |

---

## **4.0 EKFアルゴリズム実装の包括的評価**

このセクションでは、モデルからフィルタ自体に焦点を移し、実装の選択、チューニングパラメータ、および堅牢性を評価します。

### **4.1 EKFの構造とワークフロー**

コードは、標準的なEKFの2ステッププロセス（予測、更新）に正しく従っています 10。

* **予測:** x\_pred \= f(x\_est), P\_pred \= F\*P\_est\*F' \+ Q  
* **更新:** y \= z \- h(x\_pred), K \=..., x\_est \= x\_pred \+ K\*y, P\_est \=...

### **4.2 フィルタの初期化（persistent変数）**

persistent変数の使用は、繰り返し呼び出される関数内で状態を維持するための正しいMATLABの作法です。

* **初期状態 x\_est:** \[0; 0; 0; 0\] への初期化は、モータが既知の状態（静止、磁束偏差なし）から始動すると仮定した場合、安全かつ標準的な選択です。  
* **初期共分散 P\_est:** diag(\[10, 10, 1e-6, 1e-6\])。これは、電流に対する高い初期不確かさ（10 A²）と、磁束偏差に対する非常に低い初期不確かさ（1e-6 Wb²）を反映しています。これは論理的です。始動時には「偏差ゼロ」という仮定を信頼しますが、初期電流については不確かです。フィルタは最初の数回の測定に基づいて電流を迅速に収束させます。

### **4.3 共分散行列のチューニング（Q と R）**

* **観測ノイズ R:** diag(\[(0.2)^2, (0.2)^2\])。これは、電流センサノイズの標準偏差が0.2Aであることを意味し、車載グレードのセンサにとって現実的な値です。Rは、インバータを無効にした状態（ゼロ電流）で電流センサデータを記録し、ノイズの分散を計算することで経験的に決定できます 13。  
* **プロセスノイズ Q:** この行列は物理モデルへの「信頼度」を定義するため、極めて重要です。  
  * Q\_i \= (0.5)^2: これは電流予測に不確かさを注入します。モデル化されていないダイナミクス、電圧指令誤差、オイラー法による離散化誤差などを考慮します。0.5Aという値は妥当な出発点です。  
  * Q\_phi \= 1e-10: これは最も敏感なチューニングパラメータです。磁束偏差の変化率をモデル化します。この値が持つ物理的な意味を定量的に分析することが重要です。  
    1. 状態Δφはランダムウォークとしてモデル化されます。Nステップ後のランダムウォークの分散は N⋅Qϕ​ です。  
    2. N=t/Ts​ なので、時間t後の分散は (t/Ts​)⋅Qϕ​ となります。  
    3. したがって、t秒後の標準偏差（Wb単位）は (t/Ts​)⋅Qϕ​​ です。  
    4. t=60秒、Ts​=1/5000、Qϕ​=1e−10 として1分後の期待変化を計算すると、標準偏差は約0.0055 Wbとなります。  
    5. 150kWモータの永久磁石磁束は0.1〜0.2 Wb程度が一般的であり、温度による5〜10%の変化（約0.01〜0.02 Wb）は十分に考えられます。  
    6. 選択されたQ\_phiの値は、1分後にフィルタが磁束偏差を開始点から数ミリウェーバ以内に留まることを期待していることを示唆しており、これは非常に遅い変化率を仮定しています。

このQ\_phiの値は、モータ温度が急激に変化する場合（例：急加速時）には小さすぎる可能性があります。Q\_phiが小さすぎると、フィルタは「硬直化」し、実際の磁束変化への応答が遅れ、推定が遅延します 15。モータの熱的ダイナミクスに基づいてQ\_phiを調整することを推奨します。

### **4.4 共分散更新式 \- 致命的なバグの特定**

コードには以下の行が含まれています。  
P\_est \= (eye(4) \- K\*H) \* P\_pred; % Joseph form for stability  
これは、**コメントの意図と矛盾する致命的なバグ**です。この式は標準的な簡易更新式であり、真のJoseph形式ではありません。Joseph形式は、数値的に優れた以下の式で与えられます 16。

Pest​=(I−KH)Ppred​(I−KH)T+KRKT  
このバグの影響は甚大です。簡易形式は、カルマンゲインKが最適であるという仮定の下で導出されます。しかし、実世界の実装では、浮動小数点演算が微小な数値誤差を導入します。簡易形式に含まれる減算は、多数の反復を経て、P行列がその本質的な特性である**対称性**と**正定値性**を失う原因となり得ます。Pが正定値でなくなると、フィルタは「発散」し、推定値が不安定で無意味になる可能性があります。

Joseph形式は、これらの丸め誤差に対してより堅牢になるように特別に構成されています。(I−KH)P(I−KH)T の項は本質的に対称性を保持し、KRKT 項の加算が正定値性を維持するのに役立ちます。これは、特に単精度浮動小数点演算を使用する可能性のある組込みプロセッサ上でのフィルタの長期的な安定性を脅かす、最優先で修正すべき問題です。

### **表 4.1: EKFチューニングパラメータガイド**

| パラメータ | 物理的意味 | 値を大きくした場合の効果 | チューニング方法 |
| :---- | :---- | :---- | :---- |
| P\_est(0) | 状態推定の初期信頼度 | 初期知識が少ないと仮定。収束時間が長くなる。 | 不確かな状態には大きく、既知の状態には小さく設定。 |
| Q\_i | 電気モデルの不確かさ | モデルへの信頼度が低下し、観測値により追従。応答性は向上するがノイズが増加。 | イノベーション系列の分析に基づき調整。過渡応答を許容できるまで増加させる。 |
| Q\_phi | 磁束偏差の変化率（熱時定数に関連） | 磁束変化への適応が速くなるが、推定値のノイズが増加。 | 予想される熱時定数に基づき調整。ステップ応答試験で推定の遅れを確認。 |
| R | 電流センサのノイズ分散 | 観測値への信頼度が低下し、モデルにより追従。滑らかになるが応答性は低下。 | インバータOFFで電流を測定し、信号の統計的分散を計算。センサのデータシートを出発点とする。 |

---

## **5.0 コードレベルのレビューと実践的な推奨事項**

### **5.1 MATLABの構文とスタイル**

コードは構文的に正しく、適切にフォーマットされています。変数名（id\_meas, phi\_d0など）は明確で自己文書化されています。入出力の詳細な説明を含む関数ヘッダの使用は優れた実践です。

### **5.2 計算効率**

最も計算負荷の高い操作は、行列の乗算と4x4行列の逆行列計算（inv(J)）です。MATLABでは4x4行列に対するinv()の呼び出しは高度に最適化されていますが、組込みターゲット上では手作業でコーディングした逆行列計算の方が高速な場合があります。カルマンゲインKの計算には2x2行列Sの逆行列計算が含まれますが、これも非常に高速です。現在の実装はMATLABでのオフラインシミュレーションには十分高速である可能性が高いですが、リアルタイム展開のためにはターゲットDSP上でコードをプロファイリングすべきです。

### **5.3 堅牢性と数値に関する考慮事項**

重大なJoseph形式のバグとは別に、Jの逆行列計算（inv(J)）は、Jが特異または悪条件になった場合に失敗する可能性があります。これは、高度に飽和したモータの特定の動作点で発生することがあります。逆行列計算の前にJの条件数（cond(J)）をチェックし、もし条件数が過度に高い場合は、そのステップのフィルタ更新をスキップして数値的不安定性を防ぐことを推奨します。

---

## **6.0 統合と戦略的推奨事項**

この最終セクションでは、すべての調査結果を明確で優先順位付けされた行動計画に統合します。

### **6.1 調査結果の要約**

| 深刻度 | 発見事項 |
| :---- | :---- |
| **致命的（フィルタ破壊）** | 誤った共分散更新式（Joseph形式ではない）。 |
| **高（性能への影響）** | モデル化されていないRsの変化によるΔφ推定値のバイアスの可能性。Q\_phiのチューニングが応答性に不可欠。 |
| **中（堅牢性）** | J行列の特異性の可能性。オイラー離散化のトレードオフ。 |
| **低（情報提供）** | コードは構造化されており、解析的ヤコビ行列は正しい。 |

### **6.2 優先順位付き行動リスト**

1. **共分散更新の修正:** P\_est \= (eye(4) \- K\*H) \* P\_pred; を完全なJoseph形式 P\_pred\_joseph \= (eye(4) \- K\*H) \* P\_pred; P\_est \= P\_pred\_joseph \* (eye(4) \- K\*H)' \+ K\*R\*K'; に置き換える。  
2. **Rs変動への対処:** Rsに対する戦略を策定する。推奨されるアプローチは、状態ベクトルを x=T に拡張し、5x5のEKFを再導出することです。これは重要ですが、生産品質の推定器には必要な機能強化です。  
3. **Q\_phiのチューニング:** モータの熱挙動を特徴付けるテストを実施し、予想される磁束変化率に一致するようにQ\_phiを調整する。まず値を1桁上げて、フィルタの応答性を観察することから始める。  
4. **堅牢性チェックの追加:** 悪条件の動作点での数値誤差を防ぐため、逆行列計算の前にcond(J)チェックを実装する。

### **6.3 結論**

提供されたコードは、高性能な磁束推定器のための強力な基盤です。解析的なアプローチは健全であり、顕著な専門知識を示しています。共分散更新における致命的なバグを修正し、Rs変動の二次的影響を考慮することで、この実装はEVパワートレインの厳しい要件に適した、堅牢で生産準備の整ったアルゴリズムへと進化させることができます。

#### **引用文献**

1. Fast and Accurate Model of Interior Permanent-Magnet Machine for Dynamic Characterization \- MDPI, 7月 9, 2025にアクセス、 [https://www.mdpi.com/1996-1073/12/5/783](https://www.mdpi.com/1996-1073/12/5/783)  
2. PMSM (DQ0) \- Direct-quadrature-zero representation of permanent magnet synchronous machine \- MATLAB \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsmdq0.html](https://www.mathworks.com/help/sps/ref/pmsmdq0.html)  
3. Extended Kalman filter based estimations for improving speed‐sensored control performance of induction motors, 7月 9, 2025にアクセス、 [https://avesis.kayseri.edu.tr/yayin/66122494-224f-43a2-9565-8ff68d803101/extended-kalman-filter-based-estimations-for-improving-speed-sensored-control-performance-of-induction-motors/document.pdf](https://avesis.kayseri.edu.tr/yayin/66122494-224f-43a2-9565-8ff68d803101/extended-kalman-filter-based-estimations-for-improving-speed-sensored-control-performance-of-induction-motors/document.pdf)  
4. Extended Kalman Filter for Sensorless Fault Tolerant Control of PMSM with Stator Resistance Estimation \- SciSpace, 7月 9, 2025にアクセス、 [https://scispace.com/pdf/extended-kalman-filter-for-sensorless-fault-tolerant-control-3i36iwzsqg.pdf](https://scispace.com/pdf/extended-kalman-filter-for-sensorless-fault-tolerant-control-3i36iwzsqg.pdf)  
5. Online stator and rotor resistance estimations of IM by using EKF \- TRDizin, 7月 9, 2025にアクセス、 [https://search.trdizin.gov.tr/en/yayin/detay/1297678/online-stator-and-rotor-resistance-estimations-of-im-by-using-ekf](https://search.trdizin.gov.tr/en/yayin/detay/1297678/online-stator-and-rotor-resistance-estimations-of-im-by-using-ekf)  
6. Stator Resistance Estimation Using Adaptive Estimation via a Bank of Kalman Filters \- e-Publications@Marquette, 7月 9, 2025にアクセス、 [https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635\&context=electric\_fac](https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635&context=electric_fac)  
7. Discretization Order Influences on Extended Kalman Filter Estimation for Doubly-Fed Induction Generator \- Przegląd Elektrotechniczny, 7月 9, 2025にアクセス、 [http://pe.org.pl/articles/2024/2/20.pdf](http://pe.org.pl/articles/2024/2/20.pdf)  
8. Symplectic Discretization Methods for Parameter Estimation of a Nonlinear Mechanical System using an Extended Kalman Filter \- SciTePress, 7月 9, 2025にアクセス、 [https://www.scitepress.org/papers/2016/59735/59735.pdf](https://www.scitepress.org/papers/2016/59735/59735.pdf)  
9. Various Ways to Compute the Continuous-Discrete Extended Kalman Filter \- ResearchGate, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/254057379\_Various\_Ways\_to\_Compute\_the\_Continuous-Discrete\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/254057379_Various_Ways_to_Compute_the_Continuous-Discrete_Extended_Kalman_Filter)  
10. The math behind Extended Kalman Filtering | by Sasha Przybylski \- Medium, 7月 9, 2025にアクセス、 [https://medium.com/@sasha\_przybylski/the-math-behind-extended-kalman-filtering-0df981a87453](https://medium.com/@sasha_przybylski/the-math-behind-extended-kalman-filtering-0df981a87453)  
11. Extended Kalman Filters \- MATLAB & Simulink \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html](https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html)  
12. Extended Kalman Filter Basics: A Practical Deep Dive \- Number Analytics, 7月 9, 2025にアクセス、 [https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide](https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide)  
13. How to Tune a Kalman Filter: Step-by-Step Guide | JuliaHub, 7月 9, 2025にアクセス、 [https://juliahub.com/blog/how-to-tune-kalman-filter](https://juliahub.com/blog/how-to-tune-kalman-filter)  
14. How Do You Determine the R and Q Matrices of a Kalman Filter? : r/ControlTheory \- Reddit, 7月 9, 2025にアクセス、 [https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how\_do\_you\_determine\_the\_r\_and\_q\_matrices\_of\_a/](https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how_do_you_determine_the_r_and_q_matrices_of_a/)  
15. Tuning Kalman Filter to Improve State Estimation \- MATLAB & \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html](https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html)  
16. Joseph Formulation of Unscented and Quadrature Filters with Application to Consider States \- University of Texas at Austin, 7月 9, 2025にアクセス、 [https://sites.utexas.edu/renato/files/2017/04/CUKF\_ver06.pdf](https://sites.utexas.edu/renato/files/2017/04/CUKF_ver06.pdf)  
17. Joseph form | Kalman filter for professionals, 7月 9, 2025にアクセス、 [https://kalman-filter.com/joseph-form/](https://kalman-filter.com/joseph-form/)  
18. The Ensemble Kalman Filter and Friends \- Department of Meteorology \- University of Reading, 7月 9, 2025にアクセス、 [http://www.met.reading.ac.uk/\~darc/nerc\_training/reading2014/DanceEnKFNotes.pdf](http://www.met.reading.ac.uk/~darc/nerc_training/reading2014/DanceEnKFNotes.pdf)




| 深刻度     | 指摘内容                                                                                                                         | 目的／理由                                                    |
| ------- | ---------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------- |
| **致命的** | **共分散更新が Joseph 形式になっていない**<br> `P_est = (I – K H) P_pred` となっており、<br> `P_est = (I – K H) P_pred (I – K H)ᵀ + K R Kᵀ` に修正が必要 | 数値丸め誤差で P が対称・正定値でなくなり、フィルタが発散する恐れ                       |
| **高**   | **固定子抵抗 Rs を定数扱い**<br>→ 温度変化で 40 %以上動くため Δφ に誤って吸収されるリスク                                                                     | 状態に *Rs* を追加して 5×5 EKF に拡張 **または** 外部温度推定器で補正した *Rs* を入力 |
| **高**   | **磁束偏差用プロセスノイズ Q φ の再調整**                                                                                                    | 現行値 1e-10 は変化に対して硬直気味。熱時定数に合わせて 1 桁程度増減させ、遅れとノイズのバランスを確認 |
| **中**   | **J 行列（インダクタンス行列）の逆行列計算の堅牢化**<br> `cond(J)` が大きい場合は更新をスキップ／落ち着いた値へクリップ                                                       | 飽和領域で J が悪条件化しやすく、数値不安定を回避するため                           |
| **中**   | **離散化手法（前進オイラー）妥当性確認**                                                                                                       | 5 kHz なら通常問題ないが、将来サンプリング周期を変える場合の影響を評価                   |
| **低**   | Q\_i, R, 初期 P などチューニングパラメータの妥当性確認                                                                                            | イノベーション系列と実機ノイズ測定を用いて微調整                                 |
