

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


ありがとうございます。5状態EKFにおいて共分散更新をJoseph形式に変更することの意義や効果について、数値安定性や実用上のメリット・デメリットの観点から評価します。

調査結果をまとめ次第ご報告いたします。


# EKF共分散更新の形式とJoseph形式への切り替え検討

## EKFの共分散更新: 簡易形式 vs Joseph形式

拡張カルマンフィルタ（EKF）の**共分散更新式**には、主に2つの形式があります。現在用いられている「**簡易形式**」は以下のように表されます（**K**はカルマンゲイン、**H**は観測モデル行列）:

$P_{\text{est}} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,P_{\text{pred}}.$

一方、提案されている**Joseph形式**（ジョセフ形式）は、計算上は同値ですが数値的に安定な形で共分散を更新します。Joseph形式の更新式は次の通りです:

$$
\mathbf{P}_{\text{est}} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,P_{\text{pred}}\,(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\,\mathbf{R}\,\mathbf{K}^\top,
$$

ここで**R**は観測ノイズの共分散です。Joseph形式では\$(\mathbf{I}-\mathbf{K}\mathbf{H})P\_{\text{pred}}(\mathbf{I}-\mathbf{K}\mathbf{H})^\top\$と\$\mathbf{K}\mathbf{R}\mathbf{K}^\top\$の**双方の項が対称行列**となっており、理論上は上式が(I–KH)形式と等価であることが知られています。したがって**K**が最適なカルマンゲインの場合、Joseph形式は簡易形式と同一の結果を与えます。ただし、後述するように**数値計算上の取り扱い**で差が生じます。

## Joseph形式の数値的メリット（対称性・正定性の維持）

**Joseph形式の最大の利点**は、数値計算における**対称性**および**正定性**（固有値が負にならないこと）の保証です。カルマンフィルタの理論上、誤差共分散行列Pは常に対称かつ正定（ないし半正定）であるべきですが、有限精度の計算では**丸め誤差**により\$P\$が非対称になったり、正定でなくなる（負の固有値を持つ）リスクがあります。実際、**簡易形式**の更新 \$P \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})P\$ は差分計算を含むため、小さな数値差によって\$P\$の対称性が崩れたり、わずかに負の値が混入する可能性があります。このような**非対称な共分散**が現れると、再帰計算の過程でフィルタが発散する恐れがあります。Joseph形式ではこの問題に対処するため、**差分計算を2乗の形で表現**（\$(I-KH)P(I-KH)^\top\$）し、さらに**観測ノイズ寄与**を\$K R K^\top\$で加算することで、計算誤差の影響を相殺します。その結果、**数値丸めによる対称性の崩れや正定性の喪失を防ぎ**、フィルタの安定性を向上させることができます。要するに、Joseph形式は\*\*「数値的に安定な共分散更新式」\*\*として広く知られており、特に高次元（状態自由度の大きい）システムでは安定化の効果が大きいとされています。

## 小さな観測ノイズ・高精度推定時における利点

観測ノイズ**R**が非常に小さい場合（センサー精度が極めて高い場合）や、状態推定を高精度に行う場合には、Joseph形式の採用が**より重要**になります。理由は、観測ノイズが小さい＝測定を極端に信頼する状況では、カルマンゲイン**K**が大きくなり（フィルタが観測に強く依存）、共分散**P**が急激に縮小・変化しやすいためです。そうした**極限的な条件**では、わずかな数値誤差が\$P\$の正定性を損なうリスクが高まります。例えば、プロセスノイズ**Q**および観測ノイズ**R**の両方が非常に小さい設定では、共分散が急速に小さくなる（フィルタが過信状態になる）ため、丸め誤差によって**共分散行列が不定（非正定）**になり、以降のステップで**カルマンゲインがゼロに近づいて観測を無視する**ような「ブラインドスポット」現象が生じ得ます。Joseph形式やスクエアルートフィルタのような安定化手法を用いることで、**数値計算による誤差蓄積を抑え**、小ノイズ環境下でも共分散行列の正定性を維持しやすくなります。実際、ある実装報告では**Joseph形式で共分散を更新したところ、行列のコンディション悪化（ill-conditioning）が解消し、以後フィルタが安定して動作した**との経験が報告されています。このように、測定ノイズが小さい高精度センシングや、状態推定精度を極限まで高めたいケースでは、Joseph形式により**対称性と正定性を担保した更新**を行うことが有利だと考えられます。

## 低速領域（低可観測性）での安定性への影響

モータの低速回転域（例：100rpm以下）では、システムの**可観測性**が低下し、一部の状態（例えば回転角の変化量Δφなど）が観測データに反映されにくくなります。**可観測性が低い**場合、対応する状態の不確かさ（分散）は理論上**発散傾向**を示し、共分散行列Pの該当要素がどんどん大きくなる可能性があります。このような**不良可観測条件**そのものは、フィルタの設計（センサー配置やモデル改良）で根本対応すべき問題ですが、数値計算の面から言えば**Joseph形式が有用な場面**でもあります。低速域で共分散が非常に大きくなったり、または一部要素で極端な差が生じて**行列の条件数が悪化**したとしても、Joseph形式で更新していれば**数値演算によるさらなる対称性崩れや不安定化を防ぐ**ことが期待できます。言い換えると、低可観測状態ではフィルタ自体が不安定になりやすいものの、Joseph形式なら**最低限フィルタの数値的な破綻（発散）を起こしにくくする**効果があります。特に低速域でゲイン調整が必要と感じているとのことですが、Joseph形式に変えれば過渡的な**ゲイン調整の敏感さを緩和できる可能性**があります。もっとも、可観測性の問題自体を解決するわけではないため、低速での状態推定精度向上には別途対策（例えば低速域用のモード切替や、外部センサ追加など）が必要になる点には留意してください。

## 代表的な実装例における共分散更新の記述

**MATLABやSimulink**をはじめ、多くの標準的なEKF実装では**Joseph形式または同等の安定化手法**が採用されています。例えば、MathWorks社のSensor Fusion Toolbox内のEKF（`trackingEKF`）等でも内部的に安定な共分散更新が行われていると考えられます（明示的に言及はなくとも、ユーザーが数値不安定性を意識せず使えるよう設計されています）。実際、教育用や研究用のライブラリでも**Joseph形式がデフォルト**になっている例があります。Peter Corke氏のRobotics Toolboxでは「共分散更新はJoseph形式（デフォルトで有効）」と明記されており、またKalman Filterの解説書籍でも\*\*「安定化のためJoseph形式を用いるべき」**と紹介されています。さらには、米国NASAの技術資料や大学の講義ノートでも、拡張カルマンフィルタ実装時には標準の更新式ではなく**Joseph形式（またはスクエアルートフィルタ）を推奨\*\*する旨が述べられています。このように、**文献や実装の標準**としてJoseph形式は広く認識・採用されており、安定性重視の場面では事実上のデフォルトになりつつあります。

一方、簡易形式で実装される場合でも、数値安定性を高める工夫として**更新後に\$P\$の対称化（\$P \leftarrow (P+P^\top)/2\$）**を行う例もあります。しかしこれは根本解決ではなく応急処置的です。総合的に見て、代表的な実装や信頼できるソフトウェアでは**最初からJoseph形式ないしそれと同等の手法で共分散を計算**しているケースが多いと考えてよいでしょう。

## 計算コストとパフォーマンスの差異

**Joseph形式の欠点**として指摘されるのは、わずかながら**計算コスト**が増加する点です。簡易形式の更新\$P=(I-KH)P\$は行列積が一度で済みますが、Joseph形式は\$(I-KH)P(I-KH)^\top\$という二度の積と\$K R K^\top\$の加算が必要です。しかし、状態次元が5程度であればこれらの追加計算量はごく**軽微**です。典型的な5×5行列と5×mゲイン行列の積演算は現代のマイクロコントローラでも負荷にならず、実時間性への影響は無視できるでしょう。実装の手間も、式を書き換える程度で**コードの複雑さはほとんど変わりません**。むしろ既存コードにおいて外積\$K R K^\top\$の項を入れていない場合、Joseph形式に直すことで**理論式に忠実な実装**になるメリットがあります。

パフォーマンス面（推定精度や収束速度）については、**数値誤差が無視できる理想計算環境**なら簡易形式でもJoseph形式でも**理論上同一**です。したがって正常時のフィルタ推定値そのものに大きな違いは生じません。ただし、数値的に不安定な状況ではJoseph形式の方が**破綻なく推定を継続できる**ため、結果的にフィルタの実動作信頼性・ロバスト性は向上します。極端な場合、簡易形式では途中で共分散行列が不定になり計算が崩壊してしまうケースでも、Joseph形式なら**最後まで収束解を与える**ことが期待できます。総合的に見れば、計算コスト増は**ごく僅少**であり、それに対して得られる数値安定性の向上は恩恵が大きいと言えます。

## 結論: Joseph形式に変更すべきか

以上の調査を踏まえると、将来的な拡張や高精度化を見据えて**EKFの共分散更新をJoseph形式に変更することは強く推奨**できます。現時点で顕著な不安定性が出ていないとしても、フィルタに余裕を持たせ\*\*「隠れた数値問題」を予防する**意味があります。特に低速回転域など可観測性が低下する局面や、センサー精度向上による低ノイズ環境では、Joseph形式によって**安全マージンを確保**する効果が期待できます。実装コストはごく僅かで、既存コードへの変更も容易であるため、デメリットは事実上ありません。一方、仮に現在の簡易形式で**全く問題が起きておらず\*\*、システムの運用範囲でも今後問題が生じない確信がある場合は、そのままでも動作はします。しかし、EKFは長期間の運用やモデル変更時に突然数値問題が発現する可能性もあるため、安全策として**早めにJoseph形式へ統一**しておく方が賢明でしょう。

要約すると、**Joseph形式の利点**は「数値的な安定性と信頼性の向上」に尽きます。低速域でのチューニング負担軽減や将来的な高精度化にも備えられることを考えると、**Joseph形式に切り替える価値は十分にある**と言えます。現実の多くの実装例にならい、共分散更新式をJoseph形式に変更することをおすすめします。もし切り替え後に挙動が変わる懸念がある場合は、シミュレーション上で簡易形式と結果を比較し、正常に動作することを確認してから適用すると良いでしょう。結果として、フィルタの数値安定性が向上し、将来的な拡張にも耐えうる堅牢なEKF実装となるはずです。

**参考文献:** Joseph形式の概念・利点、数値安定化効果、実装上の推奨など.




# **IPM同期モータの状態・パラメータ推定：拡張カルマンフィルタの適用と高階微分の役割に関する技術レポート**

## **1\. モータ磁束マップの数値微分に関する分析**

提示されたMATLABスクリプトは、IPM（埋込磁石同期）モータのd-q軸磁束リンケージマップから1階および2階の偏導関数を計算するものです。本セクションでは、この計算アプローチの妥当性を検証し、その実装における重要な実践的課題、特にノイズと精度の問題について詳細に分析します。

### **1.1. MATLABにおける計算アプローチの妥当性検証**

提供されたスクリプトにおける微分マップの計算手法は、MATLABの標準的な実践方法に準拠しており、構文的に完全に有効です。phid\_mapのようなグリッドデータに対してgradient関数を逐次的に呼び出すことは、高階導関数を数値的に求めるための一般的なテクニックです 1。

具体的には、最初の\[Ldd, Ldq\] \= gradient(phid\_map, id\_vec, iq\_vec)という呼び出しは、磁束$\\phi\_d$の$i\_d$および$i\_q$に関する1階偏導関数の勾配成分を正しく計算します。続く\[phid\_id\_id, phid\_id\_iq\] \= gradient(Ldd, id\_vec, iq\_vec)という呼び出しは、得られた1階微分マップLdd（すなわち$\\partial\\phi\_d/\\partial i\_d$）をさらに微分し、2階偏導関数$\\partial^2\\phi\_d/\\partial i\_d^2$および$\\partial^2\\phi\_d/\\partial i\_q \\partial i\_d$を計算します。この方法は、MATLABのドキュメンテーションおよびユーザー事例によって裏付けられており、グリッド化されたデータセットに対する高階微分の計算に適しています 1。

結論として、ユーザーのコードは、意図された計算（グリッドデータ上の1階および2階の偏導関数の算出）を、MATLABの標準的な機能を用いて正しく実装しています。

### **1.2. gradient関数のメカニズム：有限差分近似**

gradient関数が実行している内容を理解することは極めて重要です。この関数は解析的な微分を行うのではなく、**数値的な**微分を実行します。具体的には、有限差分法を用いて導関数を近似します 3。MATLABの

gradient関数は、グリッドの内部点に対しては2次精度の中心差分法を、グリッドの境界点（エッジ）に対しては1次精度の片側（前方または後方）差分法を適用します 3。

例えば、2次元関数$F(x,y)$の$x$に関する偏導関数$\\partial F/\\partial x$を点$(i, j)$で近似する場合、内部点では以下の中心差分式が用いられます。

∂x∂F​​(i,j)​≈2hx​F(i,j+1)−F(i,j−1)​

ここで$h\_x$は$x$方向のグリッド間隔です。一方、左端の境界点$j=1$では、前方差分が用いられます。

∂x∂F​​(i,1)​≈hx​F(i,2)−F(i,1)​

この手法は数値解析における標準的なテクニックであり、その精度はグリッド間隔$h$に依存します 7。この  
gradient関数の内部動作を理解することは、次節で議論するその限界を把握するための基礎となります。

### **1.3. 重要な限界：ノイズ増幅と精度のトレードオフ**

この数値微分アプローチには、実践上、極めて重大な限界が存在します。それは、**ノイズの増幅**です。この現象は、特に2階微分を計算する際に顕著になります。ユーザーの磁束マップ（phid\_map, phiq\_map）は、「FEM解析や実測」に由来するため、本質的に数値計算誤差や測定ノイズを含んでいます \[User Query\]。微分、特に2階微分を計算するプロセスは、これらのノイズを大幅に増幅させ、結果として得られる微分マップの信頼性を著しく損なう可能性があります。

#### **ノイズ増幅のメカニズム**

数値微分は、近接する点の値の差を取ることで導関数を近似します。元データにノイズ$\\varepsilon$が含まれている場合を考えます。1階の中心差分近似では、ノイズの影響は以下のようになります。

2h(f(x+h)+ε1​)−(f(x−h)+ε2​)​=2hf(x+h)−f(x−h)​+2hε1​−ε2​​  
ノイズ項$(\\varepsilon\_1 \- \\varepsilon\_2)$は、小さなグリッド間隔$h$で除算されるため、その影響が増幅されます。

2階微分の近似では、この問題はさらに深刻化します。

h2(f(x+h)+ε1​)−2(f(x)+ε0​)+(f(x−h)+ε2​)​=h2f(x+h)−2f(x)+f(x−h)​+h2ε1​−2ε0​+ε2​​  
この場合、ノイズの影響は$h^2$で除算されるため、$1/h^2$のオーダーで増幅されます 8。ユーザーのコードは、このノイズを増幅するプロセスを2回連鎖させています（

元データ \-\> 1階微分（ノイズ増幅） \-\> 2階微分（さらなるノイズ増幅））。その結果、最終的に得られるヘシアンマップ（phid\_id\_idなど）は、真の2階微分の信号よりも、増幅されたノイズに支配される危険性が非常に高いです。この現象は、数値微分におけるよく知られた課題です 2。このリスクを軽減するためには、微分計算を行う前に元の磁束マップに対して平滑化フィルタを適用するなどの前処理が考えられます。

#### **グリッド間隔のジレンマ**

ユーザーが選択したグリッド間隔（id\_vecとiq\_vecのステップサイズ20A）は、微分の精度を決定する重要なハイパーパラメータです。ここには、根本的なトレードオフが存在します。

1. **打ち切り誤差（Truncation Error）**: 有限差分法はテイラー展開を有限項で打ち切る近似であるため、本質的に誤差（打ち切り誤差）を含みます。この誤差はグリッド間隔$h$のべき乗（例えば$O(h^2)$）に比例するため、$h$を小さくする（グリッドを密にする）ほど打ち切り誤差は減少します 8。  
2. **丸め誤差とノイズ増幅（Round-off and Noise Amplification Error）**: 前述の通り、ノイズの増幅効果は$1/h$や$1/h^2$に比例します。したがって、$h$を小さくすると、測定ノイズや計算上の丸め誤差の影響が急激に増大します。

この2つの誤差要因は相反する関係にあり、両者の合計を最小化する最適なグリッド間隔$h$が存在します。ユーザーが選択した20Aという間隔は、実用的な妥協点かもしれませんが、最適である保証はありません。計算された導関数の信頼性を評価するためには、グリッド間隔に対する感度分析を行うことが推奨されます。例えば、10Aや40Aのステップサイズで計算を再実行し、結果として得られる微分マップがどの程度変化するかを比較することで、計算結果の頑健性を評価できます。

## **2\. 標準的な拡張カルマンフィルタとその1次近似の基礎**

本セクションでは、標準的な拡張カルマンフィルタ（EKF）の理論的基礎を確立し、ユーザーの核心的な問いに直接答えます。すなわち、EKFが1次近似に基づく手法であることを明確にします。

### **2.1. EKFアルゴリズム：2段階の再帰的プロセス**

EKFは、再帰的な「予測」と「補正（更新）」のループで動作します。

* **予測ステップ (Prediction Step)**: システムモデルを用いて、現在の状態推定値とその誤差共分散を次の時刻へ時間発展させます。  
* **補正ステップ (Correction/Update Step)**: 最新の観測値（測定値）を用いて、予測された状態を補正します。このとき、予測値と観測値のそれぞれの不確かさ（誤差共分散）に基づいて重み付けが行われます。

この2段階のプロセスはカルマンフィルタファミリーの定義的な特徴であり 12、アルゴリズムは過去の全データ履歴ではなく、直前の状態推定値のみを必要とする再帰的な性質を持ちます 12。

### **2.2. 中核原理：1次テイラー級数による線形化**

EKFの「拡張（Extended）」という名称は、**非線形**システムを扱う能力に由来します。線形モデルを要求する標準カルマンフィルタとは異なり、EKFは非線形な状態遷移行列$f(x, u)$と観測関数$h(x)$に対応できます 14。これを実現するために、EKFは各タイムステップにおいて、現在の状態推定値の周りでこれらの非線形関数を

**1次テイラー級数展開**によって線形化します 14。このプロセスは、非線形関数を現在の状態推定点における接（超）平面で近似することと等価です 17。

### **2.3. ヤコビ行列の役割**

この1次線形化の結果として現れるのが\*\*ヤコビ行列（Jacobian Matrix）\*\*です。状態遷移ヤコビアン$F$は、状態関数$f$の各要素を状態変数$x$の各要素で偏微分して得られる行列です。同様に、観測ヤコビアン$H$は、観測関数$h$の各要素を$x$で偏微分した行列です。これらのヤコビアン$F$と$H$が、標準カルマンフィルタの線形システム行列$A$と$C$の代わりとして、EKFの計算式で使用されます。

具体的には、誤差共分散の予測ステップやカルマンゲインの計算において、これらのヤコビアンが中心的な役割を果たします 18。

Pk∣k−1​=Fk−1​Pk−1∣k−1​Fk−1T​+Qk−1​Kk​=Pk∣k−1​HkT​(Hk​Pk∣k−1​HkT​+Rk​)−1

### **2.4. ユーザーの問いへの直接的な回答**

上記の理論的背景から、ユーザーの問いに対する直接的な回答が導かれます。標準的な（カノニカルな）EKFアルゴリズムは、その定義上、1次近似に基づく拡張であるため、**2階微分（ヘシアン行列）を使用しません**。線形化プロセス全体がヤコビ行列（1階微分）のみに依存しています。

したがって、ユーザーが計算した2階微分マップ（例：phid\_id\_id）は、**標準的なEKFの実装には要求されず、また利用されません**。

この事実は、線形から非線形推定へ移行する際にしばしば生じる混乱点を明らかにします。ユーザーはシステムが非線形であり、微分が必要であると正しく認識し、1階と2階の両方の微分を丹念に計算しました。ここで伝えるべき重要な点は、最も広く使われている標準EKFは、1階微分で計算を打ち切るということです。EKFの設計思想は、線形カルマンフィルタを非線形システムに対応させるための最も単純な拡張を見出すことであり、それはヤコビアンのみを用いることで達成されます。この明確な回答を基に、後続のセクションでは、では1階微分が「何のために」必要なのか（セクション3）、そして2階微分が「何のために」使われうるのか（セクション4）を詳述します。

## **3\. IPMモータのためのEKF状態空間定式化**

本セクションでは、セクション2の理論とユーザーの具体的な応用を結びつけます。IPMモータに必要な状態空間モデルを導出し、ユーザーが計算した1階微分マップ（Ldd, Ldqなど）がEKFのフレームワークの中で具体的にどこに位置づけられるのかを明確に示します。

### **3.1. 連続時間非線形状態空間モデル**

まず、システムのダイナミクスを記述する状態空間モデルを定義します。状態ベクトル$x$は、通常、d軸電流$i\_d$、q軸電流$i\_q$、電気角速度$\\omega\_r$、電気角$\\theta\_r$で構成されます。

x=\[id​​iq​​ωr​​θr​​\]T

入力$u$はd-q軸電圧$v\_d, v\_q$です。

u=\[vd​​vq​​\]T

連続時間における状態方程式$dx/dt \= f(x, u)$は、IPMモータの基本方程式から導出されます。d-q軸の電圧方程式は以下の通りです 19。  
vd​=Rs​id​+dtdψd​​−ωr​ψq​vq​=Rs​iq​+dtdψq​​+ωr​ψd​

ここで$R\_s$は固定子抵抗です。IPMモータの重要な特徴は、磁気飽和により磁束リンケージ$\\psi\_d, \\psi\_q$が電流$i\_d, i\_q$の非線形関数となる点です 19。これはまさにユーザーが  
phid\_mapとphiq\_mapでモデル化した現象です。

ψd​=ψd​(id​,iq​)ψq​=ψq​(id​,iq​)

磁束の時間微分$d\\psi/dt$は、連鎖律を用いて以下のように展開できます。  
dtdψd​​=∂id​∂ψd​​dtdid​​+∂iq​∂ψd​​dtdiq​​dtdψq​​=∂id​∂ψq​​dtdid​​+∂iq​∂ψq​​dtdiq​​  
ここで登場する偏導関数$\\partial\\psi/\\partial i$は、**微分インダクタンス**として知られ、ユーザーが計算したLdd, Ldq, Lqd, Lqqに他なりません。これらを状態に依存するインダクタンス行列$L(i\_d, i\_q)$としてまとめます。

$$L(i\_d, i\_q) \= \\begin{bmatrix} \\frac{\\partial\\psi\_d}{\\partial i\_d} & \\frac{\\partial\\psi\_d}{\\partial i\_q} \\\\ \\frac{\\partial\\psi\_q}{\\partial i\_d} & \\frac{\\partial\\psi\_q}{\\partial i\_q} \\end{bmatrix} \= \\begin{bmatrix} L\_{dd} & L\_{dq} \\\\ L\_{qd} & L\_{qq} \\end{bmatrix} $$これを電圧方程式に代入し、電流の時間微分\`$di/dt$\`について解くと、電気系の状態方程式が得られます。$$ \\begin{bmatrix} \\frac{di\_d}{dt} \\\\ \\frac{di\_q}{dt} \\end{bmatrix} \= L(i\_d, i\_q)^{-1} \\left( \\begin{bmatrix} v\_d \\\\ v\_q \\end{bmatrix} \- R\_s \\begin{bmatrix} i\_d \\\\ i\_q \\end{bmatrix} \- \\omega\_r \\begin{bmatrix} \-\\psi\_q(i\_d, i\_q) \\\\ \\psi\_d(i\_d, i\_q) \\end{bmatrix} \\right) $$機械系のダイナミクスは、トルク方程式によって記述されます。$$ \\frac{d\\omega\_r}{dt} \= \\frac{1}{J\_m} (T\_e \- T\_L \- B\\omega\_r)$$Te​=23​p(ψd​(id​,iq​)iq​−ψq​(id​,iq​)id​)dtdθr​​=ωr​

ここで$J\_m$はイナーシャ、$T\_e$は電磁トルク、$T\_L$は負荷トルク、$B$は粘性摩擦係数、$p$は極対数です 19。これらの方程式群が、非線形な状態関数  
$f(x, u)$を構成します。

### **3.2. 状態空間モデルの離散化**

EKFは離散時間フィルタであるため、導出した連続時間モデル$dx/dt \= f(x, u)$を離散時間モデル$x\_k \= f\_d(x\_{k-1}, u\_{k-1})$に変換する必要があります 16。最も単純な方法は1次のオイラー法です。

xk​=xk−1​+Ts​⋅f(xk−1​,uk−1​)

ここで$T\_s$はサンプリング時間です。より大きなサンプリング時間で高い精度を求める場合には、ルンゲ・クッタ法のような高次の数値積分手法が有効です 22。

### **3.3. 状態遷移ヤコビアン（Fk）の導出**

このセクションが、ユーザーの計算とEKF理論を結びつける核心部分です。状態遷移ヤコビアン$F\_k$は、離散化された状態関数$f\_d$を状態$x$で偏微分し、$x\_{k-1}$で評価したものです。オイラー法を用いる場合、$F\_k$は以下のようになります。

Fk​=∂x∂fd​​​xk−1​​=I+Ts​∂x∂f​​xk−1​​  
ここで$I$は単位行列です。したがって、連続時間モデルのヤコビアン$\\partial f / \\partial x$を計算する必要があります。この$\\partial f / \\partial x$は$4 \\times 4$の行列であり、その要素は$\\partial(di\_d/dt)/\\partial i\_d$や$\\partial(di\_d/dt)/\\partial i\_q$などから構成されます。

ここで重要な点が明らかになります。電流ダイナミクス$di/dt$は、状態依存のインダクタンス行列$L(i\_d, i\_q)$の逆行列を含んでいます。したがって、$di/dt$を$i\_d$や$i\_q$で偏微分する際には、逆行列の微分の公式と積の微分法則を適用する必要があり、その計算過程で$L(i\_d, i\_q)$の各要素（$L\_{dd}, L\_{dq}$など）の$i\_d, i\_q$に関する微分、すなわち**磁束の2階微分**が必要となります。

例えば、$\\partial(di\_d/dt)/\\partial i\_d$を計算するには、$\\partial(L^{-1})/\\partial i\_d$の項と$\\partial(\\psi\_q)/\\partial i\_d (=L\_{qd})$の項が現れます。$\\partial(L^{-1})/\\partial i\_d$は、$\\partial L / \\partial i\_d$に依存し、$\\partial L / \\partial i\_d$の要素は$\\partial L\_{dd}/\\partial i\_d \= \\partial^2\\psi\_d/\\partial i\_d^2$（ユーザーのphid\_id\_id）などです。

これは、セクション2の回答をより洗練させる重要な結論です。標準的なEKFは、アルゴリズムの形式上は1階微分（ヤコビアン）しか使用しません。しかし、IPMモータのように状態変数がモデルのパラメータ（この場合は微分インダクタンス）に影響を与えるシステムでは、**状態遷移ヤコビアン$F\_k$を計算する内部で、磁束マップの2階微分が暗黙的に必要となる**のです。したがって、ユーザーが計算した2階微分マップは、EKF実装において直接的ではないものの、ヤコビアンを正確に計算するために不可欠な要素となります。

### **3.4. 観測モデルとヤコビアン（Hk）**

次に観測モデル$z\_k \= h(x\_k) \+ v\_k$を定義します。ここで$z\_k$は観測ベクトル、$v\_k$は観測ノイズです。モータ制御システムでは、通常、d-q軸電流が測定（または三相電流から変換）されます。したがって、観測ベクトルは$z\_k \= \[i\_d^{\\text{meas}}, i\_q^{\\text{meas}}\]^T$となります。

この場合、観測関数$h(x)$は状態ベクトルから観測可能な量を取り出す単純な射影となります。

$$h(x) \= \\begin{bmatrix} 1 & 0 & 0 & 0 \\\\ 0 & 1 & 0 & 0 \\end{bmatrix} x \= \\begin{bmatrix} i\_d \\\\ i\_q \\end{bmatrix} $$この観測モデルは線形であるため、そのヤコビアン\`$H\_k$\`は状態に依存しない定数行列となります \[26, 27\]。$$ H\_k \= \\frac{\\partial h}{\\partial x} \= \\begin{bmatrix} 1 & 0 & 0 & 0 \\\\ 0 & 1 & 0 & 0 \\end{bmatrix}$$  
このように、IPMモータのEKFでは、プロセスのダイナミクスは高度に非線形である一方、観測モデルは単純な線形となるのが一般的です。

### **表1：微分マップの役割一覧**

以下の表は、ユーザーが作成した微分マップの各要素が、EKFの文脈でどのような数学的・物理的意味を持ち、どのような役割を果たすかをまとめたものです。

| MATLAB変数名 | 数学的表記 | 物理的解釈 | EKFにおける役割 |
| :---- | :---- | :---- | :---- |
| derivative\_maps.Ldd | $\\partial\\psi\_d / \\partial i\_d$ | d軸微分インダクタンス | 状態依存インダクタンス行列$L(x)$の要素。非線形状態関数$f(x,u)$を定義する。 |
| derivative\_maps.Ldq | $\\partial\\psi\_d / \\partial i\_q$ | d-q軸間クロス微分インダクタンス | 状態依存インダクタンス行列$L(x)$の要素。非線形状態関数$f(x,u)$を定義する。 |
| derivative\_maps.Lqd | $\\partial\\psi\_q / \\partial i\_d$ | q-d軸間クロス微分インダクタンス | 状態依存インダクタンス行列$L(x)$の要素。非線形状態関数$f(x,u)$を定義する。 |
| derivative\_maps.Lqq | $\\partial\\psi\_q / \\partial i\_q$ | q軸微分インダクタンス | 状態依存インダクタンス行列$L(x)$の要素。非線形状態関数$f(x,u)$を定義する。 |
| derivative\_maps.phid\_id\_id | $\\partial^2\\psi\_d / \\partial i\_d^2$ | d軸微分インダクタンスの$i\_d$変化率 | 状態遷移ヤコビアン$F\_k$の計算に必要。$\\partial(L\_{dd})/\\partial i\_d$として使用される。 |
| derivative\_maps.phid\_id\_iq | $\\partial^2\\psi\_d / \\partial i\_d \\partial i\_q$ | d軸微分インダクタンスの$i\_q$変化率 | 状態遷移ヤコビアン$F\_k$の計算に必要。$\\partial(L\_{dd})/\\partial i\_q$として使用される。 |
| ... (その他も同様) | ... | ... | ... |

## **4\. 高度な推定と分析における2階微分の有用性**

標準的なEKFの実装において2階微分が（暗黙的にではあるが）必要であることが示されましたが、本セクションでは、2階微分がより直接的かつ明示的に利用される高度な手法や分析について議論します。これにより、ユーザーが計算したヘシアンマップの価値がさらに高まります。

### **4.1. 精度の向上：2次拡張カルマンフィルタ（SOEKF）**

標準EKFの1次近似は、システムの非線形性が強い場合に大きな線形化誤差を生じさせ、推定性能を低下させる可能性があります。この問題に対処するため、テイラー展開の2次項までを考慮に入れて状態と共分散の伝播を計算する\*\*2次拡張カルマンフィルタ（Second-Order EKF, SOEKF）\*\*が提案されています。

SOEKFでは、ユーザーが計算したヘシアン行列（2階偏導関数からなる行列）が、状態予測の更新式に明示的に組み込まれます。概念的には、状態予測は以下のように修正されます。

x^k∣k−1​=fd​(x^k−1∣k−1​)+21​i=1∑n​ei​⋅trace(Hfi​​Pk−1∣k−1​)  
ここで$H\_{f\_i}$は状態関数$f$の$i$番目の要素のヘシアン行列であり、$e\_i$は$i$番目の要素が1の単位ベクトルです。この追加項は、状態分布の非対称性（歪度）を考慮に入れることで、予測のバイアスを低減し、推定精度を向上させます 17。ユーザーの

phid\_id\_idマップなどは、まさにこのヘシアン行列$H\_{f\_i}$の構成要素となります。

### **4.2. 診断ツールとしての活用：線形化誤差の定量化**

SOEKFを実装しない場合でも、ヘシアン行列はEKFの性能を分析するための強力な診断ツールとして機能します。ある動作点（特定の$i\_d, i\_q$のペア）における2階微分の大きさは、その点における関数の曲率、すなわち**非線形性の強さ**を直接的に示す指標となります。

この観点から、ユーザーがすでに作成した2階微分マップの3Dプロット（例：surf(Id\_grid, Iq\_grid, derivative\_maps.phid\_id\_id)）は、単なる可視化結果以上の価値を持ちます。それは、EKFの**性能予測マップ**として解釈できます。

1. EKFの主な誤差源の一つは、ノイズを除けば線形化誤差です 14。  
2. テイラーの定理によれば、1次近似の誤差は2階微分によって評価されます。  
3. ユーザーは、この2階微分をモータの全動作領域にわたって計算しています \[User Code\]。

したがって、ヘシアン成分の3Dプロットは、事実上、線形化誤差の大きさのマップと見なすことができます。マップ上で値が高い領域は、システムが最も非線形である動作点を示しています。これらの領域では、標準EKFの線形近似が最も不正確になり、フィルタの性能が低下することが予想されます。

この分析により、ユーザーは事前に問題が発生しやすい動作領域を特定できます。例えば、特定の高トルク・高回転領域でヘシアンの値が急増する場合、その領域ではEKFの推定値が信頼できない可能性があると予測できます。この知見に基づき、その領域でのみ、よりロバストなフィルタ（Unscented Kalman Filterなど）に切り替えるといった戦略的な判断が可能になります。このように、2階微分マップは、フィルタの挙動をプロアクティブに分析し、設計を改善するための貴重な情報を提供します。

## **5\. 頑健なEKF実装とチューニングのための実践ガイド**

本セクションでは、EKFを理論から実践に移し、現実世界で安定して動作させるために不可欠な、専門的かつ経験に基づく知識を提供します。

### **5.1. 数値的安定性の確保：Joseph形式の共分散更新**

EKFの実装において頻繁に遭遇する問題は、数値的な不安定性によるフィルタの発散です。有限精度の浮動小数点演算に起因して、誤差共分散行列$P$の再帰的な更新計算の過程で、$P$が本来持つべき対称性や半正定値性（固有値が非負であること）が失われることがあります。特に、標準的な共分散更新式$P\_k \= (I \- K\_k H\_k) P\_{k|k-1}$はこの問題に対して脆弱です。

この問題は古くから知られており、その解決策として**Joseph（ジョセフ）形式**の共分散更新式を使用することが強く推奨されています 29。

Pk∣k​=(I−Kk​Hk​)Pk∣k−1​(I−Kk​Hk​)T+Kk​Rk​KkT​

この式は、最適カルマンゲインを使用する場合には数学的に標準形式と等価ですが、その構造上、丸め誤差に対してより頑健であり、$P$行列の対称性と半正定値性を維持するのに役立ちます 31。特に、組み込みシステムなど計算精度に制約がある環境や、長時間の運用が求められる場合には、Joseph形式の採用が事実上の標準となっています。

### **5.2. チューニングの科学と技術：共分散行列QとR**

プロセスノイズ共分散$Q$と観測ノイズ共分散$R$は、EKFの性能を左右する最も重要なチューニングパラメータです。

* **観測ノイズ共分散 R**: 観測（測定）における不確かさを表します。この値は、使用するセンサのデータシートに記載されたノイズ仕様や、静止状態で取得したデータの分散を計算することで、比較的容易に初期値を設定できます 34。  
* **プロセスノイズ共分散 Q**: より抽象的なパラメータで、プロセスモデルそのものに対する不確かさを表します。すなわち、実際のモータが状態方程式$dx/dt \= f(x,u)$にどれだけ正確に従うか、というモデル化誤差や未モデル化ダイナミクスを表現します 36。

$Q$と$R$のバランスが、フィルタの挙動を決定します。

* $Q$が$R$に比べて小さい場合：フィルタはモデルをより信頼し、観測ノイズの影響を強く抑制します。結果として推定値は滑らかになりますが、実際の状態変化への追従が遅れる（ラグが発生する）傾向があります。  
* $Q$が$R$に比べて大きい場合：フィルタは観測値をより信頼し、状態変化に素早く追従します。しかし、観測ノイズの影響を受けやすくなり、推定値がノイジーになります 36。

これらの行列のチューニングは、しばしば「試行錯誤」に頼らざるを得ない、EKF実装における最大の難関の一つとされています 38。

### **5.3. 試行錯誤からの脱却：イノベーション解析による系統的チューニング**

$Q$と$R$を闇雲に調整するのではなく、より系統的なアプローチが可能です。その鍵となるのが、フィルタの\*\*イノベーション（innovation）または残差（residual）\*\*を分析することです。イノベーション$\\nu\_k$は、実際の観測値$z\_k$とフィルタによる予測観測値$h(\\hat{x}\_{k|k-1})$の差として定義されます。

νk​=zk​−h(x^k∣k−1​)

統計的に最適なフィルタ（すなわち、$Q$と$R$が適切に設定されたフィルタ）では、このイノベーション系列は以下の2つの重要な特性を持つべきです 39。

1. **ゼロ平均（Zero-mean）**: 平均値が0であること。  
2. **白色性（Whiteness）**: 系列内に時間的な相関がないこと（自己相関が時間ラグ0でのみ値を持ち、その他では0であること）。

最適フィルタは、過去の情報をすべて利用して最良の予測を行うため、その予測誤差（イノベーション）は、過去の誤差から予測不可能なランダムなものであるはずです。この「予測不可能性」が白色性に対応します。

もし計算されたイノベーション系列が白色でない（例えば、自己相関に関係が見られる）場合、それはフィルタが系統的な予測誤差を犯していることを意味し、カルマンゲイン$K$が不適切である、ひいては$Q$や$R$が誤って設定されていることを示唆します 40。この特性を利用することで、チューニングを試行錯誤から科学的なプロセスへと昇華させることができます。例えば、イノベーションの自己相関をプロットし、ラグ0以外にピークが見られる場合、それはフィルタがラグっている（ゲインが低い）か、振動している（ゲインが高い）かを示唆します。これに応じて

$Q$や$R$を調整することで、イノベーションが白色に近づくようにフィルタを改善できます。

### **5.4. 定量的検証：正規化イノベーション二乗（NIS）検定**

イノベーションの白色性という定性的な評価を、より厳密な統計的仮説検定に落とし込んだものが正規化イノベーション二乗（Normalized Innovation Squared, NIS）検定です。各タイムステップ$k$におけるNIS値$\\epsilon\_\\nu$は、以下のように計算されるスカラ量です。

ϵν,k​=νkT​Sk−1​νk​

ここで$S\_k$は理論的なイノベーション共分散であり、$S\_k \= H\_k P\_{k|k-1} H\_k^T \+ R\_k$で与えられます 43。  
もしフィルタが統計的に整合性を持っている（つまり$Q, R, H$が正しい）ならば、このNIS値$\\epsilon\_\\nu$は、観測ベクトルの次元数$n\_z$を自由度とする**カイ二乗（$\\chi^2$）分布**に従います 43。この性質を利用して、例えば95%の信頼区間を設定し、計算されたNIS値がこの区間内に収まっているかを監視します。

* **NIS値が信頼区間の上限を一貫して超える場合**: フィルタが自身の予測を過信している（$S\_k$を過小評価している）ことを示します。これは通常、プロセスノイズ$Q$が小さすぎるか、観測ノイズ$R$が大きすぎることに起因します。  
* **NIS値が信頼区間の下限を一貫して下回る場合**: フィルタが過度に保守的である（$S\_k$を過大評価している）ことを示します。これは通常、$Q$が大きすぎるか、$R$が小さすぎることが原因です。

NIS検定は、フィルタの「健康状態」を定量的に監視する強力なツールです。また、類似の検定であるNEES（正規化推定誤差二乗）検定とは異なり、NIS検定は「真の状態値」を必要とせず、観測値のみで計算できるため、シミュレーションだけでなく実世界のデータにも適用可能であるという大きな利点があります 46。

### **表2：EKF実装トラブルシューティングガイド**

| 観測される症状 | 考えられる原因 | 診断テスト | 推奨される解決策 |
| :---- | :---- | :---- | :---- |
| フィルタが発散する。$P$行列が非正定値またはNaN/Infになる。 | 共分散更新における数値的不安定性。 | 各ステップで$P$行列の対称性や固有値を確認する。 | Joseph形式の共分散更新式（セクション5.1）を実装する。モデルのヤコビアンが正確であることを再確認する。 |
| 推定値が非常にノイジーで、観測ノイズに過敏に反応する。 | フィルタがモデルよりも観測値を過信している。$Q$が大きすぎるか、$R$が小さすぎる。 | NIS検定を実施する。平均NISがカイ二乗信頼区間の下限を下回る可能性が高い。 | 観測ノイズ共分散行列$R$の対角成分を系統的に増加させる。または、プロセスノイズ$Q$を減少させる。 |
| 推定値が真値に対して著しく遅れる（ラグが大きい）。 | フィルタが観測値よりもモデルを過信している。$Q$が小さすぎるか、$R$が大きすぎる。 | イノベーションの自己相関をプロットする。正の相関がラグ1以降で見られる。平均NISが信頼区間の上限を超える可能性が高い。 | プロセスノイズ共分散行列$Q$の対角成分を系統的に増加させる。または、観測ノイズ$R$を減少させる。 |
| 推定値に定常的なバイアス（偏り）が見られる。 | モデルにバイアスが存在する（例：$R\_s$のモデル値が不正確）。または、センサにバイアスがある。 | イノベーション系列の平均値がゼロから有意にずれているかを確認する。 | モデルのパラメータ（例：$R\_s$）を状態ベクトルに含めて同時に推定する（パラメータ推定EKF）。センサのキャリブレーションを再確認する。 |

## **6\. 統合と戦略的提言**

本レポートの分析結果を統合し、ユーザーが次に進むべき明確かつ戦略的な道筋を提示します。

### **6.1. 分析結果の要約**

* **計算手法の妥当性**: ユーザーがMATLABで実装した1階および2階微分の計算方法は、構文的に正しく、標準的なアプローチです。  
* **1階微分の役割**: ユーザーが計算した1階微分マップ（Ldd, Ldqなど）は、微分インダクタンスとして、IPMモータの非線形状態空間モデルを定義し、そのヤコビアン$F\_k$を導出するために**不可欠**です。  
* **2階微分の役割**: ユーザーが計算した2階微分マップ（phid\_id\_idなど）は、標準的なEKFのアルゴリズムには**直接使用されません**。しかし、状態遷移ヤコビアン$F\_k$を正確に計算する過程で**間接的に必要**となります。さらに、これらはより高度な2次EKF（SOEKF）で明示的に使用されるほか、システムの非線形性を評価し、標準EKFの性能限界を予測するための強力な**診断ツール**として極めて有用です。  
* **実践上の課題**: FEMや実測データからの数値微分は、ノイズを著しく増幅させるという重大な実践的課題を伴います。結果の信頼性には常に注意が必要です。

### **6.2. 推奨される実装戦略**

以上の分析に基づき、堅牢な状態推定器を構築するための段階的な戦略を以下に推奨します。

1. **ステップ1：標準EKFに集中する**: まずは高次のフィルタを検討する前に、頑健な標準EKFを実装することに全力を注ぎます。  
2. **ステップ2：正確な状態空間モデルの構築**: セクション3で詳述した通り、IPMモータの非線形な状態関数$f(x,u)と観測関数$h(x)を注意深く構築します。特に、微分マップが状態遷移ヤコビアン$F\_k$の計算にどのように寄与するかを正確に理解し、実装します。  
3. **ステップ3：数値的安定性の確保**: 実装の初期段階から、セクション5.1で推奨したJoseph形式の共分散更新式を採用し、一般的な発散の問題を未然に防ぎます。  
4. **ステップ4：系統的なチューニングと検証**: $Q$と$R$のチューニングに際して、手作業による試行錯誤に依存しないでください。セクション5.3で説明したイノベーション系列の分析を指針とし、セクション5.4のNIS検定をフィルタの整合性を定量的に検証し、チューニングを導くためのツールとして実装・活用します。  
5. **ステップ5（将来的な作業）：高度な手法の探求**: 頑健な標準EKFが動作するようになった後、セクション4.2で述べたように、2階微分マップを診断ツールとして利用し、非線形性が特に強い動作領域を特定します。もしこれらの領域で推定性能が不十分であると判断された場合に限り、2次EKFやUnscented Kalman Filter（UKF）のような、より計算コストの高い、しかし精度の高いフィルタの実装を検討します。

この戦略的なアプローチにより、理論的な正確さと実践的な堅牢性を両立させた、高性能な状態推定器の開発が可能となります。

#### **引用文献**

1. 2nd order gradient \- MATLAB Answers \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/matlabcentral/answers/832833-2nd-order-gradient](https://www.mathworks.com/matlabcentral/answers/832833-2nd-order-gradient)  
2. How to take first and second order derivative of discrete data? \- MATLAB Answers, 7月 10, 2025にアクセス、 [https://www.mathworks.com/matlabcentral/answers/1879472-how-to-take-first-and-second-order-derivative-of-discrete-data](https://www.mathworks.com/matlabcentral/answers/1879472-how-to-take-first-and-second-order-derivative-of-discrete-data)  
3. Numerical gradient \- MATLAB \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/help/matlab/ref/gradient.html](https://www.mathworks.com/help/matlab/ref/gradient.html)  
4. gradient (MATLAB Functions), 7月 10, 2025にアクセス、 [http://www.ece.northwestern.edu/local-apps/matlabhelp/techdoc/ref/gradient.html](http://www.ece.northwestern.edu/local-apps/matlabhelp/techdoc/ref/gradient.html)  
5. What is the definition of the GRADIENT function in MATLAB 6.5 (R13)? \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/matlabcentral/answers/101820-what-is-the-definition-of-the-gradient-function-in-matlab-6-5-r13](https://www.mathworks.com/matlabcentral/answers/101820-what-is-the-definition-of-the-gradient-function-in-matlab-6-5-r13)  
6. MatLab \- gradient command \- Stack Overflow, 7月 10, 2025にアクセス、 [https://stackoverflow.com/questions/11767015/matlab-gradient-command](https://stackoverflow.com/questions/11767015/matlab-gradient-command)  
7. Finite difference method \- Wikipedia, 7月 10, 2025にアクセス、 [https://en.wikipedia.org/wiki/Finite\_difference\_method](https://en.wikipedia.org/wiki/Finite_difference_method)  
8. 6: Finite Difference Approximation \- Mathematics LibreTexts, 7月 10, 2025にアクセス、 [https://math.libretexts.org/Bookshelves/Scientific\_Computing\_Simulations\_and\_Modeling/Scientific\_Computing\_(Chasnov)/I%3A\_Numerical\_Methods/6%3A\_Finite\_Difference\_Approximation](https://math.libretexts.org/Bookshelves/Scientific_Computing_Simulations_and_Modeling/Scientific_Computing_\(Chasnov\)/I%3A_Numerical_Methods/6%3A_Finite_Difference_Approximation)  
9. 4.2. Finite difference method — Mechanical Engineering Methods, 7月 10, 2025にアクセス、 [https://kyleniemeyer.github.io/ME373-book/content/bvps/finite-difference.html](https://kyleniemeyer.github.io/ME373-book/content/bvps/finite-difference.html)  
10. Is numeric gradient estimation well-posed \- MATLAB Answers \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/matlabcentral/answers/181568-is-numeric-gradient-estimation-well-posed](https://www.mathworks.com/matlabcentral/answers/181568-is-numeric-gradient-estimation-well-posed)  
11. Numerical Differentiation, 7月 10, 2025にアクセス、 [https://www.sheffield.ac.uk/media/32080/download?attachment](https://www.sheffield.ac.uk/media/32080/download?attachment)  
12. Kalman filter \- Wikipedia, 7月 10, 2025にアクセス、 [https://en.wikipedia.org/wiki/Kalman\_filter](https://en.wikipedia.org/wiki/Kalman_filter)  
13. Kalman filter \- Wikipedia, the free encyclopedia \- NYU Stern, 7月 9, 2025にアクセス、 [https://pages.stern.nyu.edu/\~dbackus/Identification/Kalman\_filter\_Wikipedia\_May10.pdf](https://pages.stern.nyu.edu/~dbackus/Identification/Kalman_filter_Wikipedia_May10.pdf)  
14. Extended Kalman Filter Basics: A Practical Deep Dive \- Number Analytics, 7月 10, 2025にアクセス、 [https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide](https://www.numberanalytics.com/blog/extended-kalman-filter-practical-guide)  
15. Application of an extended kalman filter for stator fault diagnosis of ..., 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/292378098\_Application\_of\_an\_extended\_kalman\_filter\_for\_stator\_fault\_diagnosis\_of\_the\_induction\_motor](https://www.researchgate.net/publication/292378098_Application_of_an_extended_kalman_filter_for_stator_fault_diagnosis_of_the_induction_motor)  
16. Extended Kalman Filter — AHRS 0.4.0 documentation, 7月 9, 2025にアクセス、 [https://ahrs.readthedocs.io/en/latest/filters/ekf.html](https://ahrs.readthedocs.io/en/latest/filters/ekf.html)  
17. Lecture Notes No. 8 \[ ( ), 7月 10, 2025にアクセス、 [https://ocw.mit.edu/courses/2-160-identification-estimation-and-learning-spring-2006/5d7c37b93786ddb91cd913d1ab994848\_lecture\_8.pdf](https://ocw.mit.edu/courses/2-160-identification-estimation-and-learning-spring-2006/5d7c37b93786ddb91cd913d1ab994848_lecture_8.pdf)  
18. Extended Kalman Filters \- MATLAB & Simulink \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html](https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html)  
19. Fast and Accurate Model of Interior Permanent-Magnet Machine for Dynamic Characterization \- MDPI, 7月 9, 2025にアクセス、 [https://www.mdpi.com/1996-1073/12/5/783](https://www.mdpi.com/1996-1073/12/5/783)  
20. PMSM (DQ0) \- Direct-quadrature-zero representation of permanent magnet synchronous machine \- MATLAB \- MathWorks, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsmdq0.html](https://www.mathworks.com/help/sps/ref/pmsmdq0.html)  
21. PMSM \- Permanent magnet synchronous motor with sinusoidal flux distribution \- MATLAB, 7月 9, 2025にアクセス、 [https://www.mathworks.com/help/sps/ref/pmsm.html](https://www.mathworks.com/help/sps/ref/pmsm.html)  
22. Discretization Order Influences on Extended Kalman Filter Estimation for Doubly-Fed Induction Generator \- Przegląd Elektrotechniczny, 7月 9, 2025にアクセス、 [http://pe.org.pl/articles/2024/2/20.pdf](http://pe.org.pl/articles/2024/2/20.pdf)  
23. Symplectic Discretization Methods for Parameter Estimation of a Nonlinear Mechanical System using an Extended Kalman Filter \- SciTePress, 7月 9, 2025にアクセス、 [https://www.scitepress.org/papers/2016/59735/59735.pdf](https://www.scitepress.org/papers/2016/59735/59735.pdf)  
24. Various Ways to Compute the Continuous-Discrete Extended Kalman Filter \- ResearchGate, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/254057379\_Various\_Ways\_to\_Compute\_the\_Continuous-Discrete\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/254057379_Various_Ways_to_Compute_the_Continuous-Discrete_Extended_Kalman_Filter)  
25. Accurate Numerical Implementation of the Continuous-Discrete Extended Kalman Filter | Request PDF \- ResearchGate, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/260711450\_Accurate\_Numerical\_Implementation\_of\_the\_Continuous-Discrete\_Extended\_Kalman\_Filter](https://www.researchgate.net/publication/260711450_Accurate_Numerical_Implementation_of_the_Continuous-Discrete_Extended_Kalman_Filter)  
26. (PDF) Extended Kalman Filter tuning in sensorless PMSM drives \- ResearchGate, 7月 10, 2025にアクセス、 [https://www.researchgate.net/publication/3171693\_Extended\_Kalman\_Filter\_tuning\_in\_sensorless\_PMSM\_drives](https://www.researchgate.net/publication/3171693_Extended_Kalman_Filter_tuning_in_sensorless_PMSM_drives)  
27. Tuning of Extended Kalman Filter for nonlinear State Estimation \- IOSR Journal, 7月 10, 2025にアクセス、 [https://www.iosrjournals.org/iosr-jce/papers/Vol18-issue5/Version-4/C1805041419.pdf](https://www.iosrjournals.org/iosr-jce/papers/Vol18-issue5/Version-4/C1805041419.pdf)  
28. Joseph covariance formula adaptation to Square-Root Sigma-Point Kalman filters \- POLITECNICO DI TORINO Repository ISTITUZIONALE, 7月 9, 2025にアクセス、 [https://iris.polito.it/retrieve/handle/11583/2657935/149622/Joseph%20covariance%20formula%20adaptation%20to%20Square-RootSigma-Point\_postprintdraft.pdf](https://iris.polito.it/retrieve/handle/11583/2657935/149622/Joseph%20covariance%20formula%20adaptation%20to%20Square-RootSigma-Point_postprintdraft.pdf)  
29. Joseph form | Kalman filter for professionals, 7月 10, 2025にアクセス、 [https://kalman-filter.com/joseph-form/](https://kalman-filter.com/joseph-form/)  
30. Numerical Instability Kalman Filter in MatLab \- Stack Overflow, 7月 10, 2025にアクセス、 [https://stackoverflow.com/questions/29459158/numerical-instability-kalman-filter-in-matlab](https://stackoverflow.com/questions/29459158/numerical-instability-kalman-filter-in-matlab)  
31. Joseph Formulation of Unscented and Quadrature Filters with Application to Consider States \- University of Texas at Austin, 7月 10, 2025にアクセス、 [https://sites.utexas.edu/renato/files/2017/04/CUKF\_ver06.pdf](https://sites.utexas.edu/renato/files/2017/04/CUKF_ver06.pdf)  
32. The Ensemble Kalman Filter and Friends \- Department of Meteorology \- University of Reading, 7月 9, 2025にアクセス、 [http://www.met.reading.ac.uk/\~darc/nerc\_training/reading2014/DanceEnKFNotes.pdf](http://www.met.reading.ac.uk/~darc/nerc_training/reading2014/DanceEnKFNotes.pdf)  
33. (PDF) Joseph Formulation of Unscented and Quadrature Filters with ..., 7月 10, 2025にアクセス、 [https://www.researchgate.net/publication/260466467\_Joseph\_Formulation\_of\_Unscented\_and\_Quadrature\_Filters\_with\_Application\_to\_Consider\_States](https://www.researchgate.net/publication/260466467_Joseph_Formulation_of_Unscented_and_Quadrature_Filters_with_Application_to_Consider_States)  
34. How to Tune a Kalman Filter: Step-by-Step Guide | JuliaHub, 7月 9, 2025にアクセス、 [https://juliahub.com/blog/how-to-tune-kalman-filter](https://juliahub.com/blog/how-to-tune-kalman-filter)  
35. How to compute the process and measurement noise covariance in extended Kalman filter, 7月 9, 2025にアクセス、 [https://www.quora.com/How-can-I-compute-the-process-and-measurement-noise-covariance-in-extended-Kalman-filter](https://www.quora.com/How-can-I-compute-the-process-and-measurement-noise-covariance-in-extended-Kalman-filter)  
36. Tuning Kalman Filter to Improve State Estimation \- MATLAB & \- MathWorks, 7月 10, 2025にアクセス、 [https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html](https://www.mathworks.com/help/fusion/ug/tuning-kalman-filter-to-improve-state-estimation.html)  
37. Kalman Filter Tutorial, 7月 9, 2025にアクセス、 [https://www.kalmanfilter.net/](https://www.kalmanfilter.net/)  
38. Adaptive Adjustment of Noise Covariance in Kalman Filter for Dynamic State Estimation, 7月 9, 2025にアクセス、 [https://www.researchgate.net/publication/313365845\_Adaptive\_Adjustment\_of\_Noise\_Covariance\_in\_Kalman\_Filter\_for\_Dynamic\_State\_Estimation](https://www.researchgate.net/publication/313365845_Adaptive_Adjustment_of_Noise_Covariance_in_Kalman_Filter_for_Dynamic_State_Estimation)  
39. Bayesian Optimization for Fine-Tuning EKF Parameters in UAV ..., 7月 10, 2025にアクセス、 [https://www.mdpi.com/2226-4310/10/12/1023](https://www.mdpi.com/2226-4310/10/12/1023)  
40. A Modified Whiteness Test for Damage Detection Using Kalman Filter Innovations, 7月 10, 2025にアクセス、 [https://www.researchgate.net/publication/226975932\_A\_Modified\_Whiteness\_Test\_for\_Damage\_Detection\_Using\_Kalman\_Filter\_Innovations](https://www.researchgate.net/publication/226975932_A_Modified_Whiteness_Test_for_Damage_Detection_Using_Kalman_Filter_Innovations)  
41. Kalman and Particle Filtering \- University of Pennsylvania, 7月 10, 2025にアクセス、 [https://www.sas.upenn.edu/\~jesusfv/filters\_format.pdf](https://www.sas.upenn.edu/~jesusfv/filters_format.pdf)  
42. Applied kalman filter theory \- Duke People, 7月 10, 2025にアクセス、 [https://people.duke.edu/\~hpgavin/SystemID/References/Balut-KalmanFilter-PhD-NEU-2011.pdf](https://people.duke.edu/~hpgavin/SystemID/References/Balut-KalmanFilter-PhD-NEU-2011.pdf)  
43. Normalized Innovation Squared (NIS) | Kalman filter for professionals, 7月 10, 2025にアクセス、 [https://kalman-filter.com/normalized-innovation-squared/](https://kalman-filter.com/normalized-innovation-squared/)  
44. NIS of Adaptive Filter, PF, UKF, and EKF respectively. \- ResearchGate, 7月 10, 2025にアクセス、 [https://www.researchgate.net/figure/NIS-of-Adaptive-Filter-PF-UKF-and-EKF-respectively\_fig3\_235042517](https://www.researchgate.net/figure/NIS-of-Adaptive-Filter-PF-UKF-and-EKF-respectively_fig3_235042517)  
45. For an ideal Kalman filter, I have that the NEES test passes but NIS test does not?, 7月 10, 2025にアクセス、 [https://stats.stackexchange.com/questions/649403/for-an-ideal-kalman-filter-i-have-that-the-nees-test-passes-but-nis-test-does-n](https://stats.stackexchange.com/questions/649403/for-an-ideal-kalman-filter-i-have-that-the-nees-test-passes-but-nis-test-does-n)  
46. Normalized Estimation Error Squared (NEES) | Kalman filter for ..., 7月 10, 2025にアクセス、 [https://kalman-filter.com/normalized-estimation-error-squared/](https://kalman-filter.com/normalized-estimation-error-squared/)


