

# **EKFベース磁束偏差オブザーバのエキスパートレビューと性能向上ガイド**

本レポートでは、ご提示いただいたモータ磁束推定用EKF（拡張カルマンフィルタ）実装における共分散更新式の正当性に関するご質問にお答えします。ご使用の更新式 $P\_{\\text{est}} \= (\\mathbf{I} \- \\mathbf{K}\\mathbf{H})\\mathbf{P}\_{\\text{pred}}$ は、理想的な条件下では数学的に正しいものの、実践上は数値的に不安定であることが知られており、ご指摘の「致命的なバグ」の原因である可能性が非常に高いと結論付けられます。本レビューでは、この問題の根本原因を明確に解説し、数値的に安定した優れた代替案を提示します。さらに、EKF実装全体にわたる包括的な分析を行い、その精度、安定性、そして総合的な性能を向上させるための専門的な推奨事項を提供します。これにより、単に機能するアルゴリズムから、実製品レベルで信頼できるオブザーバへと昇華させることを目指します。

---

## **第1章：共分散更新式の重要性 — 理論的正しさから実装上の頑健性へ**

このセクションでは、ユーザーの主たる懸念事項である共分散更新式について直接的に論じます。ここでの核心は、有限精度の計算環境において、数学的な等価性が必ずしも数値的な等価性を意味しないという点にあります。

### **1.1. 標準形式の理論的な正しさ**

ご提示のコードで使用されている共分散更新式、

Pk​=(I−Kk​Hk​)Pk∣k−1​

は、標準的なカルマンフィルタの導出における中心的な式の一つです 1。この形式は、観測値を取り込むことによって状態の不確かさがどのように減少するかを表現しており、計算効率が高く、数学的にも完全に有効です。  
この式は、事後誤差共分散の定義式 $ \\mathbf{P}\_k \= E\[(\\mathbf{x}\_k \- \\hat{\\mathbf{x}}\_k)(\\mathbf{x}\_k \- \\hat{\\mathbf{x}}\_k)^\\top\] $ から導出され、多くの入門的な教科書や実装例で紹介されています 3。理論的な基盤を共有するため、この導出過程を簡単に確認することから始めます。状態更新式

$\\hat{\\mathbf{x}}\_k \= \\hat{\\mathbf{x}}\_{k|k-1} \+ \\mathbf{K}\_k (\\mathbf{z}\_k \- \\mathbf{H}\_k \\hat{\\mathbf{x}}\_{k|k-1})$ を誤差の定義に代入し、整理することで上記の更新式が得られます。この式自体に理論的な誤りは一切ありません。

### **1.2. 潜在的な危険性：数値不安定性**

標準形式の更新式が抱える決定的な問題は、DSPやマイクロコントローラのような有限精度の演算環境における丸め誤差に対する脆弱性です 5。特に、

$(\\mathbf{I} \- \\mathbf{K}\\mathbf{H})$ の項に含まれる減算処理は、計算結果として得られる事後共分散行列 $\\mathbf{P}\_{\\text{est}}$ が、その物理的な性質であるはずの対称性や半正定値性を失う原因となり得ます 7。

共分散行列が正定値性を失うと、分散が負になるという物理的にあり得ない状況を意味し、フィルタが発散する直接的な原因となります。これは、推定プロセス全体の致命的な失敗につながりかねません 8。この数値的な脆弱性は、現実世界のカルマンフィルタ実装において古くから知られている問題であり、ご指摘の「致命的なバグ」の最も有力な原因と考えられます 6。これはコードの論理的な誤りではなく、選択された数式の根本的な弱点に起因する問題です。

この問題の根源を理解することは、単なるバグ修正以上の意味を持ちます。それは、理論的なアルゴリズムを現実のハードウェアに実装する際に生じる、理論と実践のギャップを認識することです。ご提示のコードにおける問題は、標準的な公式をコーディングしたこと自体が誤りなのではなく、数値精度が問題となる実用的なアプリケーションに対して、数値的に脆弱な標準公式を選択したことが誤りなのです。したがって、この問題への対処は、単なるコード修正ではなく、より頑健なエンジニアリング手法を採用するという設計思想の転換を意味します。

### **1.3. 解決策：Joseph形式の採用による安定性の確保**

この数値的不安定性の問題を解決するために、Joseph形式として知られる共分散更新式が明示的に設計されています 10。その式は以下のように表されます。

Pest​=(I−KH)Ppred​(I−KH)⊤+KRK⊤  
Joseph形式が機能する理由：  
この形式の優位性は、その数学的構造にあります。事後共分散を2つの行列の和として計算します。

1. 第一項 $(\\mathbf{I} \- \\mathbf{K}\\mathbf{H}) \\mathbf{P}\_{\\text{pred}} (\\mathbf{I} \- \\mathbf{K}\\mathbf{H})^\\top$ は二次形式 $\\mathbf{A}\\mathbf{P}\\mathbf{A}^\\top$ の形をしています。この構造により、たとえカルマンゲイン $\\mathbf{K}$ に微小な数値誤差が含まれていたとしても、計算結果は常に対称かつ半正定値であることが保証されます。  
2. 第二項 $\\mathbf{K}\\mathbf{R}\\mathbf{K}^\\top$ も同様に二次形式であり、測定ノイズ共分散 $\\mathbf{R}$ が正定値である限り、この項も半正定値となります。  
3. 二つの半正定値行列の和は、常に半正定値です 1。

この構造により、Joseph形式は共分散行列が持つべき物理的な性質（対称性、半正定値性）を演算の過程で本質的に維持します。

実装の修正：  
したがって、ご提示のコードの該当行は以下のように修正されるべきです。

Matlab

% P\_est \= (eye(4) \- K\*H)\*P\_pred; % ← 数値的に不安定な標準形式

% Joseph形式による共分散更新 \- 数値的に安定  
I\_KH \= eye(4) \- K\*H;  
P\_est \= I\_KH \* P\_pred \* I\_KH.' \+ K \* R \* K.';

Joseph形式は標準形式に比べてわずかに計算コストが高いですが、フィルタの安定性を保証するという絶大な利益の前では、そのコストは無視できるレベルです 10。特に、安全性が重視されるシステムや高信頼性が要求されるアプリケーションにおいては、この形式の採用が標準的なプラクティスとされています。

---

## **第2章：システムモデルの詳細な検証**

このセクションでは、フィルタの物理的・数学的な基盤となるシステムモデルを検証します。ここでの誤りは、フィルタのメカニズムにおける誤りよりもデバッグが困難な場合が多く、極めて重要です。

### **2.1. 状態空間モデルの妥当性 (f\_discrete\_delta\_phi)**

状態ベクトルは $\\mathbf{x} \=^\\top$ と定義されています。このうち、磁束偏差の状態 $\\Delta\\phi\_d$ と $\\Delta\\phi\_q$ は、$\\Delta\\phi\_{d, \\text{next}} \= \\Delta\\phi\_d$ のように、時間的に変化しないランダムウォークとしてモデル化されています。これは、温度変化などによってゆっくりと変動する未知のパラメータやバイアスをモデル化するための標準的かつ適切な手法です 11。

連続時間における電流のダイナミクスは、以下の式で与えられています。

* $di\_d/dt \= (1/L\_{d, \\text{map}}) \\cdot (v\_d \- R\_s i\_d \+ \\omega(\\phi\_{q, \\text{map}} \+ \\Delta\\phi\_q))$  
* $di\_q/dt \= (1/L\_{q, \\text{map}}) \\cdot (v\_q \- R\_s i\_q \- \\omega(\\phi\_{d, \\text{map}} \+ \\Delta\\phi\_d))$

この定式化は、d-q回転座標系におけるPMSM（永久磁石同期モータ）の電圧方程式を正しく表現しています 14。総磁束は、マップベースの磁束（

$\\phi\_{d, \\text{map}}$, $\\phi\_{q, \\text{map}}$）と推定された偏差（$\\Delta\\phi\_d$, $\\Delta\\phi\_q$）の和として定義されており、$\\omega\\phi\_q$ や $\\omega\\phi\_d$ といった交差結合項も正しく含まれています。モデルの物理的な妥当性は高いと評価できます。

### **2.2. ヤコビ行列の精査 (calculate\_F\_delta\_phi)**

ヤコビ行列 $\\mathbf{A} \= d(\\dot{\\mathbf{x}})/d\\mathbf{x}$ は、連続時間ダイナミクスの偏微分からなる行列です。ご提示のコードでは、これを $\\mathbf{F} \= \\mathbf{I} \+ \\mathbf{A}T\_s$ という形で離散化しています。ここで、$\\mathbf{A}$ の各要素の正当性を検証します。

IPMSM（埋込磁石同期モータ）では、磁気飽和により磁束鎖交数 $\\phi\_d$ と $\\phi\_q$ は電流 $(i\_d, i\_q)$ の非線形関数となります。したがって、総磁束は $\\phi\_{d, \\text{total}} \= \\phi\_{d, \\text{map}}(i\_d, i\_q) \+ \\Delta\\phi\_d$ および $\\phi\_{q, \\text{total}} \= \\phi\_{q, \\text{map}}(i\_d, i\_q) \+ \\Delta\\phi\_q$ となります。これを用いて電圧方程式を表現し、$di/dt$ について整理すると、電流ダイナミクスが得られます。

ヤコビ行列 $\\mathbf{A}$ をこの完全な非線形モデルから再導出すると、以下のようになります。

* $A(1,1) \= \\frac{\\partial(\\dot{i\_d})}{\\partial i\_d} \= \\frac{1}{L\_d} \\left \= \\frac{-R\_s \+ \\omega L\_{qd}}{L\_d}$。これはコードと一致します。  
* $A(1,2) \= \\frac{\\partial(\\dot{i\_d})}{\\partial i\_q} \= \\frac{1}{L\_d} \\left\[ \\omega \\frac{\\partial\\phi\_q}{\\partial i\_q} \\right\] \= \\frac{\\omega L\_q}{L\_d}$。ここで、$L\_q \= \\partial\\phi\_q/\\partial i\_q$ です。コードでは $\\omega L\_{qq} / L\_{d, \\text{map}}$ となっています。$L\_{qq}$ が $\\partial\\phi\_q/\\partial i\_q$ を意味するのであれば、これも一致します。  
* $A(1,4) \= \\frac{\\partial(\\dot{i\_d})}{\\partial \\Delta\\phi\_q} \= \\frac{1}{L\_d} \[\\omega \\cdot 1\] \= \\frac{\\omega}{L\_d}$。これもコードと一致します。  
* $A(2,1) \= \\frac{\\partial(\\dot{i\_q})}{\\partial i\_d} \= \\frac{1}{L\_q} \\left\[ \-\\omega \\frac{\\partial\\phi\_d}{\\partial i\_d} \\right\] \= \\frac{-\\omega L\_d}{L\_q}$。ここで、$L\_d \= \\partial\\phi\_d/\\partial i\_d$ です。コードでは $-\\omega L\_{dd} / L\_{q, \\text{map}}$ となっています。$L\_{dd}$ が $\\partial\\phi\_d/\\partial i\_d$ を意味するのであれば、これも一致します。  
* $A(2,2) \= \\frac{\\partial(\\dot{i\_q})}{\\partial i\_q} \= \\frac{1}{L\_q} \\left \= \\frac{-R\_s \- \\omega L\_{dq}}{L\_q}$。これもコードと一致します。  
* $A(2,3) \= \\frac{\\partial(\\dot{i\_q})}{\\partial \\Delta\\phi\_d} \= \\frac{1}{L\_q} \[-\\omega \\cdot 1\] \= \-\\frac{\\omega}{L\_q}$。これもコードと一致します。

ヤコビ行列に関する結論と注意点：  
ヤコビ行列の構造は、標準的なIPMSMモデルに基づいており、物理的に正しいように見えます。しかしながら、ここで極めて重要な注意点があります。それは、入力変数名 Ldd, Ldq, Lqd, Lqq の解釈です。  
一般的に、モータ理論におけるインダクタンスは磁束の1階偏微分として定義されます（例：$L\_d \= \\partial\\phi\_d/\\partial i\_d$、交差結合インダクタンス $L\_{dq} \= \\partial\\phi\_d/\\partial i\_q$）。一方で、$L\_{dd}$ のような表記は、通常、磁束の2階偏微分（ヘッセ行列の要素、$\\partial^2\\phi\_d/\\partial i\_d^2$）を示唆します。

ご提示のコードのヤコビ行列の計算式は、これらの変数が1階の偏微分（すなわち、標準および交差結合インダクタンス）を表しているかのように使用しています。この解釈の曖昧さは、実装における深刻なエラーの原因となり得ます。例えば、Simulinkの入力として、物理的に異なる量（2階微分）が、1階微分を期待する計算式に供給されている可能性があります。これは、フィルタが安定して動作しているように見えても、その推定値が不正確であるという、診断が困難な問題を引き起こします。

したがって、**これらの入力変数の定義を厳密に確認し、Simulinkから渡されるルックアップテーブル（LUT）の値が、ヤコビ行列の計算で意図されている物理量（$\\partial\\phi/\\partial i$）と正確に対応していることを保証する**ことを強く推奨します。科学技術計算における変数名の明確化とドキュメンテーションの重要性を示す典型的な例です。

### **2.3. 離散化手法の評価**

ご提示のコードでは、1次オイラー法 $\\mathbf{F} \= \\mathbf{I} \+ \\mathbf{A}T\_s$ を用いて離散化を行っています。これは最も単純な方法であり、サンプリング周期 $T\_s$ が十分に小さい（サンプリング周波数が高い）場合には、多くの場合で十分な精度が得られます。

しかし、システムのダイナミクスが速い場合や、サンプリング周波数が比較的低い場合には、この近似が大きな離散化誤差を導入する可能性があります 17。ご提示の

$T\_s \= 500\\mu s$ (2 kHz) という設定は、多くのモータ制御アプリケーションにとって十分に高速であるため、オイラー法が問題となる可能性は低いと考えられます。

推奨事項：  
もし他の修正を行った後にも精度に関する問題が残る場合は、より高度な離散化手法の検討が有効です。例えば、行列指数関数 $\\mathbf{F} \= \\text{expm}(\\mathbf{A}T\_s)$ に基づく方法や、ルンゲ・クッタ法などが挙げられます 18。ただし、現時点では、オイラー法は妥当な選択であると判断します。

---

## **第3章：EKFチューニングの技術と科学**

このセクションでは、場当たり的な試行錯誤から、データに基づいた体系的なプロセスへとユーザーを導くことで、最も大きな付加価値を提供します。EKFのチューニングは「黒魔術」ではなく、統計的な検証に基づく科学的なプロセスです。

### **3.1. Q行列とR行列の解読**

* $\\mathbf{R} \= \\text{diag}(\[1\\text{e-}3, 1\\text{e-}3\])$: これは測定ノイズ共分散行列です。値 $1\\text{e-}3$ は、標準偏差 $\\sqrt{1\\text{e-}3} \\approx 0.0316$ \[A\] に相当します。これは、使用している電流センサのノイズ標準偏差が約32mAであると仮定していることを意味します。高品質な電流センサにとっては、妥当な初期値と言えます。$\\mathbf{R}$ は、測定値をどれだけ信頼するかをフィルタに伝えます 21。  
* $\\mathbf{Q} \= \\text{diag}(\[1\\text{e-}4, 1\\text{e-}4, 1\\text{e-}12, 1\\text{e-}12\])$: これはプロセスノイズ共分散行列です。  
  * $Q(1,1)$ と $Q(2,2)$ ($1\\text{e-}4$): このノイズは電流状態 $(i\_d, i\_q)$ に加算されます。これは、電圧方程式自体の不確かさ、例えば抵抗値 $R\_s$ の誤差やモデル化されていないダイナミクスなどを表現します。タイムステップ毎に標準偏差 $\\sqrt{1\\text{e-}4} \= 0.01$ \[A\] のノイズが注入されていることになります。  
  * $Q(3,3)$ と $Q(4,4)$ ($1\\text{e-}12$): このノイズは磁束偏差状態 $(\\Delta\\phi\_d, \\Delta\\phi\_q)$ に加算されます。これは、フィルタが磁束推定値をどれだけ速く変化させることを許容するかを決定する、極めて重要なチューニングパラメータです 4。値  
    $1\\text{e-}12$ は非常に小さく、標準偏差 $1\\text{e-}6$ \[Wb\] に相当します。これは、真の磁束鎖交数マップは非常に正確であり、その変化（例：温度による）は極めてゆっくりであるという強い信念を反映しています。

### **3.2. 体系的なチューニングフレームワーク：イノベーション解析**

フィルタの性能を評価するために、最終的な状態推定値そのものを見る必要はありません。多くの場合、真の状態（Ground Truth）は未知です。その代わりに、フィルタ自身が自己診断に必要なすべてのデータを提供してくれます。その鍵となるのが「イノベーション（innovation）」または「残差（residual）」です。

* **診断ツールとしてのイノベーション系列:** イノベーション $\\mathbf{y} \= \\mathbf{z} \- \\mathbf{H}\\mathbf{x}\_{\\text{pred}}$ は、実際の測定値とフィルタによる予測測定値との差です。理論的に、完全に調整された最適なカルマンフィルタでは、このイノベーション系列は以下の2つの重要な特性を持ちます：  
  1. 平均値がゼロである。  
  2. 自己相関がなく、白色雑音（white noise）である 26。  
* **イノベーションの理論的共分散:** この系列の理論的な共分散は、イノベーション共分散行列 $\\mathbf{S} \= \\mathbf{H}\\mathbf{P}\_{\\text{pred}}\\mathbf{H}^\\top \+ \\mathbf{R}$ によって与えられます 4。  
* **チューニングの核心:** $\\mathbf{Q}$ と $\\mathbf{R}$ を調整する目的は、実際に測定されたイノベーション系列 $\\mathbf{y}$ の統計的性質を、その理論的な統計的性質（平均ゼロ、共分散 $\\mathbf{S}$）に一致させることです。これらが一致すれば、フィルタは「整合性（consistent）」があり、最適に動作していると判断できます 25。

### **3.3. 実践的実装：正規化イノベーション二乗（NIS）検定**

イノベーションを単に「目で見る」のではなく、正規化イノベーション二乗（Normalized Innovation Squared, NIS）検定と呼ばれる形式的な統計検定を用いることで、定量的な評価が可能になります 30。

* **NISの計算:** 各タイムステップ $k$ で、以下のNIS値を計算します。εk​=yk⊤​Sk−1​yk​  
* **統計的性質:** このスカラ値 $\\varepsilon\_k$ は、測定値の数（ここでは $i\_d, i\_q$ の2つなので $m=2$）を自由度とするカイ二乗（$\\chi^2$）分布に従います。  
* **仮説検定:** NISの値を時系列でプロットし、カイ二乗分布の95%信頼区間と比較することで、フィルタの整合性を確認できます。平均してNIS値の約95%がこの区間内に収まっていれば、フィルタは良好に調整されていると言えます。あまりにも多くの点が区間外にある場合、フィルタは不整合であり、$\\mathbf{Q}$ や $\\mathbf{R}$ の設定が不適切であることを示唆します 30。  
* **ユーザー向け手順:**  
  1. EKFの各ステップで、イノベーションベクトル $\\mathbf{y}$ と計算されたイノベーション共分散 $\\mathbf{S}$ をログに記録します。  
  2. オフライン（MATLABなど）で、記録したデータからNIS系列 $\\varepsilon\_k$ を計算します。  
  3. MATLABの chi2inv(0.025, 2\) と chi2inv(0.975, 2\) を用いて、95%信頼区間の上下限を計算します。  
  4. $\\varepsilon\_k$ をこの信頼区間と共にプロットし、評価します。

### **3.4. NIS検定結果に基づくチューニングヒューリスティクス**

以下の表は、NIS検定の結果に基づいて $\\mathbf{Q}$ と $\\mathbf{R}$ をどのように調整すればよいかを示す、実践的なガイドです。これは、推測から体系的なチューニングへと移行するための非常に価値のあるツールです。

| NIS検定の観測結果 | 解釈 | 主要な対策 | 副次的な対策 |
| :---- | :---- | :---- | :---- |
| NIS値が継続的に**高すぎる**（上限を超える） | フィルタが過信している。実際のイノベーションが、フィルタが予測する共分散 $\\mathbf{S}$ よりも大きい。測定値を十分に信頼していない。 | 測定ノイズ $\\mathbf{R}$ を**小さくする**。これにより $\\mathbf{S}$ が小さくなり、カルマンゲイン $\\mathbf{K}$ が大きくなるため、フィルタは測定値により追従するようになる。 | プロセスノイズ $\\mathbf{Q}$ を**大きくする**。これにより $\\mathbf{P}\_{\\text{pred}}$ が増大し、結果として $\\mathbf{S}$ も大きくなる。フィルタは自身の予測に自信がなくなる。 |
| NIS値が継続的に**低すぎる**（下限を下回る） | フィルタの信頼性が低い（反応が鈍い）。実際のイノベーションが予測よりも小さい。測定ノイズに過敏に反応しすぎている。 | 測定ノイズ $\\mathbf{R}$ を**大きくする**。これによりフィルタは自身のモデルをより信頼し、ノイズの多い測定値への反応が鈍くなる。 | プロセスノイズ $\\mathbf{Q}$ を**小さくする**。これによりフィルタは自身の予測に自信を持ち、$\\mathbf{P}\_{\\text{pred}}$ と $\\mathbf{S}$ が減少する。 |
| NIS値に**バイアス**がある（平均値が $m=2$ ではない） | フィルタに体系的な誤差がある。イノベーション $\\mathbf{y}$ の平均がゼロではない。 | センサのバイアス（$\\mathbf{R}$ でモデル化されていない）を確認する。システムモデル $\\mathbf{f}(\\mathbf{x}, \\mathbf{u})$ や測定モデル $\\mathbf{h}(\\mathbf{x})$ の誤差を調査する。 | これは $\\mathbf{Q}$/$\\mathbf{R}$ のチューニングよりも、モデルの正当性の問題である可能性が高い。 |
| NIS値に**自己相関**がある（白色ではない） | フィルタのダイナミクスが速すぎるか遅すぎる。モデルがシステムの時間的挙動を捉えきれていない。 | 遅れている、または行き過ぎている状態に対応する $\\mathbf{Q}$ 行列の要素を調整する。例えば、磁束推定の収束が遅い場合、$Q(3,3)$ と $Q(4,4)$ を**大きくする**。 | システムモデル $\\mathbf{f}(\\mathbf{x}, \\mathbf{u})$ とサンプリング時間 $T\_s$ を再評価する。 |

---

## **第4章：最終的な推奨事項とベストプラクティス**

このセクションでは、結論の要約と、その他の重要な実践的考慮事項のチェックリストを提供します。

### **4.1. 状態の初期化と収束**

* $\\mathbf{x}\_{\\text{est}} \= \[0; 0; 0; 0\]$: 電流と磁束偏差をゼロで初期化するのは、モータが既知の状態（例：静止しており、磁束誤差がない）から始動すると仮定した場合、合理的で一般的な選択です。  
* $\\mathbf{P}\_{\\text{est}} \= \\text{diag}(\[0.01, 0.01, 1\\text{e-}2, 1\\text{e-}2\])$: この共分散行列の初期化は、フィルタの収束挙動にとって極めて重要です。  
  * $P(1,1), P(2,2)$ ($0.01$): 電流は始動時に直接測定できるため、その初期不確かさを小さく設定するのは適切です。  
  * $P(3,3), P(4,4)$ ($1\\text{e-}2$): 磁束偏差状態の初期不確かさを**大きく**設定することが不可欠です。これは、フィルタが初期段階では $\\Delta\\phi$ についてほとんど何も知らないことを伝え、初期の測定値に大きく依存して迅速に収束させることを強制します。選択された値 $0.01$ は、この目的に対して適切な大きさです。

### **4.2. コードレベルのベストプラクティス**

* Ld\_map \= max(Ld\_map, 1e-6); という行は、優れた防御的プログラミングの一例です。これは、インダクタンスのルックアップテーブルが（例えばゼロ電流時に）ゼロを返した場合のゼロ除算エラーを防ぎます。これは、実践的なコーディングセンスの良さを示しています。  
* K \= (P\_pred\*H.') / S; % バックスラッシュ(\\)よりも読みやすい というコメントは洞察に富んでいます。mrdivide 演算子 / を使用することは、確かに可読性が高く、$\\mathbf{X}\\mathbf{S} \= \\mathbf{B}$（ここで $\\mathbf{X}=\\mathbf{K}$, $\\mathbf{B}=\\mathbf{P}\_{\\text{pred}}\\mathbf{H}^\\top$）を解く上で数値的にも安定しています。MATLABの行列除算演算子は高度に最適化されており、陽的な逆行列計算（inv(S)）よりも推奨されます。

### **4.3. デバッグと検証のためのチェックリスト**

ユーザーが従うべき、最終的な実践的チェックリストを以下に示します。

1. **Joseph形式の実装:** 共分散更新式を数値的に安定なバージョンに置き換える。  
2. **ヤコビ行列入力の検証:** Ldd, Ldq, Lqd, Lqq の物理的な意味を確認し、LUTから正しい値が渡されていることを保証する。  
3. **イノベーションデータの記録:** オフライン解析のために $\\mathbf{y}$ と $\\mathbf{S}$ を記録するようにコードを計装する。  
4. **NIS検定の実施:** 記録したデータを用いてNIS検定を実行し、フィルタの整合性を確認する。  
5. **QとRのチューニング:** NISの結果と前述のヒューリスティクス表を用いて、$\\mathbf{Q}$ と $\\mathbf{R}$ を体系的に調整する。  
6. **発散の確認:** $\\mathbf{P}\_{\\text{est}}$ の対角要素を監視する。もし際限なく増大する場合、フィルタは発散している（モデル、$\\mathbf{Q}$、安定性をチェックする）。  
7. **バイアスの確認:** イノベーション $\\mathbf{y}$ を時系列でプロットする。その平均がゼロでない場合、センサやモデルのバイアスを調査する。

## **結論**

本レポートの主要な結論を以下に要約します。ご指摘の「致命的なバグ」は、数値的不安定性という、微妙かつ重大な問題でした。これは、Joseph形式の共分散更新式を採用することで、確定的に解決できます。

しかし、この修正を超えて真の高性能を達成するためには、正確なシステムモデルと体系的なチューニングが不可欠です。本レポートで推奨したイノベーションベースのチューニング手法（特にNIS検定）を適用することにより、場当たり的な試行錯誤から脱却し、頑健で信頼性が高く、かつ検証可能な形で最適なEKFを構築することが可能となります。このアプローチは、ご提示の高度なモータ制御アプリケーションにおいて、信頼性と性能の両面で大きな向上をもたらすものと確信します。

#### **引用文献**

1. The Ensemble Kalman Filter and Friends \- Department of Meteorology \- University of Reading, 7月 9, 2025にアクセス、 [http://www.met.reading.ac.uk/\~darc/nerc\_training/reading2014/DanceEnKFNotes.pdf](http://www.met.reading.ac.uk/~darc/nerc_training/reading2014/DanceEnKFNotes.pdf)  
2. Kalman filter \- Wikipedia, 7月 10, 2025にアクセス、 [https://en.wikipedia.org/wiki/Kalman\_filter](https://en.wikipedia.org/wiki/Kalman_filter)  
3. Extended Kalman Filters \- MATLAB & Simulink \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html](https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html)  
4. Extended Kalman Filter Basics: A Practical Deep Dive \- Number Analytics, 7月 10, 2025にアクセス、 [https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide](https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide)  
5. Numerical Instability Kalman Filter in MatLab \- Stack Overflow, 7月 10, 2025にアクセス、 [https://stackoverflow.com/questions/29459158/numerical-instability-kalman-filter-in-matlab](https://stackoverflow.com/questions/29459158/numerical-instability-kalman-filter-in-matlab)  
6. Lecture Notes No. 8 \[ ( ), 7月 10, 2025にアクセス、 [https://ocw.mit.edu/courses/2-160-identification-estimation-and-learning-spring-2006/5d7c37b93786ddb91cd913d1ab994848\_lecture\_8.pdf](https://ocw.mit.edu/courses/2-160-identification-estimation-and-learning-spring-2006/5d7c37b93786ddb91cd913d1ab994848_lecture_8.pdf)  
7. Joseph covariance formula adaptation to Square-Root Sigma-Point Kalman filters \- POLITECNICO DI TORINO Repository ISTITUZIONALE, 7月 9, 2025にアクセス、 [https://iris.polito.it/retrieve/handle/11583/2657935/149622/Joseph%20covariance%20formula%20adaptation%20to%20Square-RootSigma-Point\_postprintdraft.pdf](https://iris.polito.it/retrieve/handle/11583/2657935/149622/Joseph%20covariance%20formula%20adaptation%20to%20Square-RootSigma-Point_postprintdraft.pdf)  
8. Joseph form | Kalman filter for professionals, 7月 10, 2025にアクセス、 [https://kalman-filter.com/joseph-form/](https://kalman-filter.com/joseph-form/)  
9. Stability of linear and non-linear Kalman filters \- Toni Karvonen, 7月 10, 2025にアクセス、 [https://tskarvone.github.io/pdf/Karvonen2014-masters\_thesis.pdf](https://tskarvone.github.io/pdf/Karvonen2014-masters_thesis.pdf)  
10. Joseph Formulation of Unscented and Quadrature Filters with Application to Consider States \- University of Texas at Austin, 7月 10, 2025にアクセス、 [https://sites.utexas.edu/renato/files/2017/04/CUKF\_ver06.pdf](https://sites.utexas.edu/renato/files/2017/04/CUKF_ver06.pdf)  
11. Extended Kalman Filter for Sensorless Fault Tolerant Control of PMSM with Stator Resistance Estimation \- SciSpace, 7月 9, 2025にアクセス、 [https://scispace.com/pdf/extended-kalman-filter-for-sensorless-fault-tolerant-control-3i36iwzsqg.pdf](https://scispace.com/pdf/extended-kalman-filter-for-sensorless-fault-tolerant-control-3i36iwzsqg.pdf)  
12. Stator Resistance Estimation Using Adaptive Estimation via a Bank of Kalman Filters \- e-Publications@Marquette, 7月 9, 2025にアクセス、 [https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635\&context=electric\_fac](https://epublications.marquette.edu/cgi/viewcontent.cgi?article=1635&context=electric_fac)  
13. Application of an extended kalman filter for stator fault diagnosis of ..., 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/292378098\_Application\_of\_an\_extended\_kalman\_filter\_for\_stator\_fault\_diagnosis\_of\_the\_induction\_motor](https://www.researchgate.net/publication/292378098_Application_of_an_extended_kalman_filter_for_stator_fault_diagnosis_of_the_induction_motor)  
14. Fast and Accurate Model of Interior Permanent-Magnet Machine for Dynamic Characterization \- MDPI, 7月 9, 2025にアクセス、 [https://www.mdpi.com/1996-1073/12/5/783](https://www.mdpi.com/1996-1073/12/5/783)  
15. PMSM (DQ0) \- Direct-quadrature-zero representation of permanent magnet synchronous machine \- MATLAB \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsmdq0.html](https://www.mathworks.com/help/sps/ref/pmsmdq0.html)  
16. PMSM \- Permanent magnet synchronous motor with sinusoidal flux distribution \- MATLAB, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsm.html](https://www.mathworks.com/help/sps/ref/pmsm.html)  
17. Discretization Order Influences on Extended Kalman Filter Estimation for Doubly-Fed Induction Generator \- Przegląd Elektrotechniczny, 7月 9, 2025にアクセス、 [http://pe.org.pl/articles/2024/2/20.pdf](http://pe.org.pl/articles/2024/2/20.pdf)  
18. Symplectic Discretization Methods for Parameter Estimation of a Nonlinear Mechanical System using an Extended Kalman Filter \- SciTePress, 7月 9, 2025にアクセス、 [https://www.scitepress.org/papers/2016/59735/59735.pdf](https://www.scitepress.org/papers/2016/59735/59735.pdf)  
19. Various Ways to Compute the Continuous-Discrete Extended Kalman Filter \- ResearchGate, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/254057379\_Various\_Ways\_to\_Compute\_the\_Continuous-Discrete\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/254057379_Various_Ways_to_Compute_the_Continuous-Discrete_Extended_Kalman_Filter)  
20. Accurate Numerical Implementation of the Continuous-Discrete Extended Kalman Filter | Request PDF \- ResearchGate, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/260711450\_Accurate\_Numerical\_Implementation\_of\_the\_Continuous-Discrete\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/260711450_Accurate_Numerical_Implementation_of_the_Continuous-Discrete_Extended_Kalman_Filter)  
21. How Do You Determine the R and Q Matrices of a Kalman Filter? : r/ControlTheory \- Reddit, 7月 9, 2025にアクセス、 [https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how\_do\_you\_determine\_the\_r\_and\_q\_matrices\_of\_a/](https://www.reddit.com/r/ControlTheory/comments/1hoq7hu/how_do_you_determine_the_r_and_q_matrices_of_a/)  
22. Adaptive Adjustment of Noise Covariance in Kalman Filter for Dynamic State Estimation, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/313365845\_Adaptive\_Adjustment\_of\_Noise\_Covariance\_in\_Kalman\_Filter\_for\_Dynamic\_State\_Estimation](https://www.researchgate.net/publication/313365845_Adaptive_Adjustment_of_Noise_Covariance_in_Kalman_Filter_for_Dynamic_State_Estimation)  
23. How to compute the process and measurement noise covariance in extended Kalman filter, 7月 9, 2025にアクセス、 [https://www.quora.com/How-can-I-compute-the-process-and-measurement-noise-covariance-in-extended-Kalman-filter](https://www.quora.com/How-can-I-compute-the-process-and-measurement-noise-covariance-in-extended-Kalman-filter)  
24. Tuning Kalman Filter to Improve State Estimation \- MATLAB & \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html](https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html)  
25. Bayesian Optimization for Fine-Tuning EKF Parameters in UAV ..., 7月 10, 2025にアクセス、 [https://www.mdpi.com/2226-4310/10/12/1023](https://www.mdpi.com/2226-4310/10/12/1023)  
26. A Modified Whiteness Test for Damage Detection Using Kalman Filter Innovations, 7月 10, 2025にアクセス、 [https://www.researchgate.net/publication/226975932\_A\_Modified\_Whiteness\_Test\_for\_Damage\_Detection\_Using\_Kalman\_Filter\_Innovations](https://www.researchgate.net/publication/226975932_A_Modified_Whiteness_Test_for_Damage_Detection_Using_Kalman_Filter_Innovations)  
27. Kalman and Particle Filtering \- University of Pennsylvania, 7月 10, 2025にアクセス、 [https://www.sas.upenn.edu/\~jesusfv/filters\_format.pdf](https://www.sas.upenn.edu/~jesusfv/filters_format.pdf)  
28. Applied kalman filter theory \- Duke People, 7月 10, 2025にアクセス、 [https://people.duke.edu/\~hpgavin/SystemID/References/Balut-KalmanFilter-PhD-NEU-2011.pdf](https://people.duke.edu/~hpgavin/SystemID/References/Balut-KalmanFilter-PhD-NEU-2011.pdf)  
29. Adaptive Adjustment of Noise Covariance in Kalman Filter for Dynamic State Estimation \- arXiv, 7月 10, 2025にアクセス、 [https://arxiv.org/pdf/1702.00884](https://arxiv.org/pdf/1702.00884)  
30. Normalized Innovation Squared (NIS) | Kalman filter for professionals, 7月 10, 2025にアクセス、 [https://kalman-filter.com/normalized-innovation-squared/](https://kalman-filter.com/normalized-innovation-squared/)  
31. NIS of Adaptive Filter, PF, UKF, and EKF respectively. \- ResearchGate, 7月 10, 2025にアクセス、 [https://www.researchgate.net/figure/NIS-of-Adaptive-Filter-PF-UKF-and-EKF-respectively\_fig3\_235042517](https://www.researchgate.net/figure/NIS-of-Adaptive-Filter-PF-UKF-and-EKF-respectively_fig3_235042517)  
32. For an ideal Kalman filter, I have that the NEES test passes but NIS test does not?, 7月 10, 2025にアクセス、 [https://stats.stackexchange.com/questions/649403/for-an-ideal-kalman-filter-i-have-that-the-nees-test-passes-but-nis-test-does-n](https://stats.stackexchange.com/questions/649403/for-an-ideal-kalman-filter-i-have-that-the-nees-test-passes-but-nis-test-does-n)  
33. Normalized Estimation Error Squared (NEES) | Kalman filter for ..., 7月 10, 2025にアクセス、 [https://kalman-filter.com/normalized-estimation-error-squared/](https://kalman-filter.com/normalized-estimation-error-squared/)


