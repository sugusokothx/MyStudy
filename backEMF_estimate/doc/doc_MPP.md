## 1. 「効率最適 (MPP)」とは何か

制御目標が **高いトルク密度** ではなく **高い電気‐機械効率 (η)** になると，
d 軸電流 $i_d$ は **「最大トルク／アンペア (MTPA) 軌跡」** よりもさらに電流損失・鉄損・磁束関連損失を同時に最小化する値にずれます。
多くの文献ではこの効率最適点を **Maximum-Efficiency or Maximum-Power-Point (MPP)** と呼びます。

| 軌跡   | 最適化対象                                                             | 損失モデルに含める項                                  |
| ---- | ----------------------------------------------------------------- | ------------------------------------------- |
| MTPA | 銅損 $P_{\text{Cu}}$ のみ                                             | $P_{\text{Cu}}=\tfrac32 R_s(i_d^2+i_q^2)$   |
| MPP  | 総損失 $P_{\text{loss}}=P_{\text{Cu}}+P_{\text{Fe}}+P_{\text{misc}}$ | - 銅損<br>- 鉄損（ヒステリシス＋渦電流）<br>- PM 減磁損・機械損 など |

---

## 2. 等価回路と効率の式

> ここでは Interior-PMSM (IPMSM) を想定し，磁気飽和は無視します（含める場合は感インダクタ $L_d,L_q$ を電流依存に拡張）。

### 2.1 電圧モデル（d–q 座標）

$$
\begin{aligned}
v_d &= R_s i_d - \omega_e L_q i_q \\
v_q &= R_s i_q + \omega_e \Bigl(L_d i_d + \psi_f\Bigr)
\end{aligned}
$$

### 2.2 トルク

$$
T_e \;=\; \frac{3}{2}p\bigl[\psi_f i_q + (L_d-L_q)i_d i_q\bigr]
$$

### 2.3 損失

$$
\begin{aligned}
P_{\text{Cu}} &= \tfrac32 R_s\bigl(i_d^2+i_q^2\bigr) \\
P_{\text{Fe}} &= k_h f B^2 + k_e f^2 B^2 
             \;\;\;\Longrightarrow\; k_f\left(v_d^2+v_q^2\right) \\
P_{\text{misc}} &\approx P_{\text{windage}} + P_{\text{bearing}} \;\;(\text{速度のみ依存})
\end{aligned}
$$

ここで $k_f$ は実機試験や FEM から同定します。
したがって

$$
P_{\text{loss}}(i_d,i_q)=\tfrac32 R_s(i_d^2+i_q^2)+k_f (v_d^2+v_q^2)+P_{\text{misc}}(\omega_m)
$$

---

## 3. 「効率最適 d 軸電流」の導出

**与えられた出力トルク $T^\star$**・速度 $\omega_e$ で効率

$$
\eta = \frac{T^\star\,\omega_m}{T^\star\,\omega_m + P_{\text{loss}}(i_d,i_q)}
$$

を最大化する ⇒ 損失最小化問題に帰着：

$$
\min_{i_d,i_q}\;P_{\text{loss}}(i_d,i_q)
\quad
\text{s.t. } T_e(i_d,i_q)=T^\star
$$

### 3.1 解析的近似（鉄損を線形化）

1. トルク制約から $i_q$ を $i_d$ の関数へ代入

   $$
   i_q(i_d)=\frac{2T^\star}{3p\bigl[\psi_f+(L_d-L_q)i_d\bigr]}
   $$
2. それを $P_{\text{loss}}\bigl(i_d,i_q(i_d)\bigr)$ に代入し

   $$
   \frac{dP_{\text{loss}}}{d i_d}=0
   $$

   を解く。

   * 銅損だけ ⇒ **MTPA**：解析解
   * 鉄損を含む ⇒ 四次方程式程度になり，数値解が現実的

### 3.2 数値最適化フロー（実機パラメータで LUT 作成）

```python
import numpy as np
from scipy.optimize import minimize_scalar

def id_mpp(T_star, w_e, Rs, Ld, Lq, psi_f, kf):
    """与えトルク・速度の効率最適 Id を返す（Iq はトルク制約から計算）"""
    p = 4               # 極対数の例
    def loss(id_):
        iq_ = 2*T_star / (1.5*p*(psi_f + (Ld-Lq)*id_))
        vd = Rs*id_ - w_e*Lq*iq_
        vq = Rs*iq_ + w_e*(Ld*id_ + psi_f)
        Pcu = 1.5*Rs*(id_**2 + iq_**2)
        Pfe = kf*(vd**2 + vq**2)
        return Pcu + Pfe        # 速度依存損は一定項なので無視
    res = minimize_scalar(loss, bounds=(-psi_f/Ld, 0), method="bounded")
    id_opt = res.x
    iq_opt = 2*T_star / (1.5*p*(psi_f + (Ld-Lq)*id_opt))
    return id_opt, iq_opt
```

* **ルックアップテーブル化**
  オフラインで $(T^\star, \omega_e)$ グリッドを走査 → 2-D LUT
  現場では補間 (`interp2`) で取り出し，電流制御ループに供給。

---

## 4. MPP 軌跡と他の最適化軌跡の比較

| 項目    | MTPA          | MPP (提案)            | Field-Weakening (FW) |      |      |
| ----- | ------------- | ------------------- | -------------------- | ---- | ---- |
| 目的    | 電流最小 → 銅損低減   | **総損失最小 → 効率最大**    | 電圧上限満足 & 定格超速度       |      |      |
| d-軸電流 | $i_d<0$ (逆磁化) | MTPA より浅い逆磁化 or 正磁化 | 強い逆磁化 ((             | i\_d | ) 大) |
| 効果    | 低速高負荷で有利      | 広範囲で η 向上 (とくに中速)   | 高速域で電圧確保             |      |      |
| 実装    | 銅損モデルだけで簡単    | 鉄損パラメータ同定が必要        | 速度・電圧計算のみ            |      |      |

---

## 5. 実機導入のポイント

1. **鉄損パラメータ同定**

   * 定点試験 or ステップ応答で $k_f$ を推定
   * FEM ベースで周波数依存モデルを作り，式にフィット

2. **LUT 外挿安全策**

   * 高速・高トルク端では FW にスムーズに遷移
   * $i_d$ が極端になる場合は電流・電圧リミットを優先

3. **オンライン適応**（高級機能）
   巻線温度変化による $R_s$ 変動をオンライン推定 (EKF 等) し，LUT を補正

---

## 6. まとめ

* **d 軸電流指令 = MPP** は「銅損だけでなく鉄損も含めた総損失」を最小化するための値。
* 解析式でおおよそ導けるが，実用上は **速度・トルクの 2-D LUT** が一般的。
* 低速では MTPA と大差ないが **中速域で特に効率向上**。
* 実装には **鉄損パラメータの同定** と **リミット処理** が鍵。

これで **効率最適 d 軸電流 (MPP) の背景・導出・実装方法** の全体像を把握いただけるはずです。必要に応じて FEM 結果や実機試験データを組み合わせ，LUT を精緻化してみてください。
