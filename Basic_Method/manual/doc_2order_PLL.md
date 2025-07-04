# どうして `int_err` から **ω̂<sub>e</sub>** を計算しているのか？

---

## 1. ループの役割分担を整理する

| ブロック | 連続時間の伝達関数 | 役割 | 位相誤差を 0 にするために必要な積分器数 |
|----------|------------------|------|----------------------------------------|
| 位相比較器 (PD) | *e* = $I_{h\delta}$ ∝ $\sin\!\Delta\theta$ | 位相誤差 $\Delta\theta$ → 誤差信号 *e* | 0 |
| ループフィルタ (PI) | $F(s)=K_p+K_i/s$ | 誤差 *e* → 周波数補正 $\Delta\omega$ | **+1** |
| VCO / NCO | $1/s$ | 周波数 → 位相 | **+1** |

結果として **2 個の積分器**（PI 内部 1 個 + VCO 1 個）を直列に持つ **2 次 PLL** になる。

> **ポイント**  
> 誤差 → 周波数補正 → 位相、という 2 段階設計なので  
> **PI 内で積分した量は「位相」ではなく「周波数（角速度）補正」に変換して使う**。

---

## 2. 離散実装での変数の次元を確認

| 変数 | 数式 | 単位 | 意味 |
|------|------|------|------|
| `err` | $e_k = I_{h\delta,k}$ | A (電流) | 位相誤差に比例する観測量 |
| `int_err` | $\displaystyle \sum e_k T_s$ | A · s | **誤差の積分値** |
| `omega_est` | $K_p e_k + K_i \,\text{int\_err}$ | rad/s | **推定角速度 (VCO 制御量)** |
| `theta_est` | $\displaystyle \sum \omega_{\text{est},k} T_s$ | rad | 推定位相 |

* $K_p$ [rad/(A·s)]、$K_i$ [rad/(A·s²)] を設定すれば `omega_est` は正しい角速度になる。  
* `theta_est` は **ω̂<sub>e</sub>** をさらに積分（NCO 部）して得る。

---

## 3. `int_err → θ̂` に直結すると？

`int_err` をそのまま θ̂ に使うと **積分器が 1 段しか無い 1 次 PLL** となり，  
速度オフセットやドリフトがあると定常位相誤差を消せない。

---

## 4. コードの流れを式でまとめる

離散時間 $k$ ステップで

$$
\begin{aligned}
e_k &= I_{h\delta,k}\\[4pt]
I_k &= I_{k-1} + e_k T_s \quad (\text{int\_err})\\[4pt]
\hat\omega_{e,k} &= K_p\,e_k + K_i\,I_k\\[4pt]
\hat\theta_{e,k} &= \hat\theta_{e,k-1} + \hat\omega_{e,k}\,T_s
\end{aligned}
$$

* $I_k$ — PI フィルタ内の **位相誤差の積分**  
* $\hat\omega_{e,k}$ — **NCO に与える角速度補正値**  
* $\hat\theta_{e,k}$ — **最終的な位相推定値**

---

## 5. まとめ

* `int_err` は「位相誤差の時間積分」を保持するだけ。  
* **Ki** を掛けて **rad/s** にスケール変換し，**Kp** と加算して  
  **推定角速度 ω̂<sub>e</sub>** を得る。  
* その角速度を **もう一度積分** して初めて **θ̂** になる。  
* 積分器 2 段の **2 次 PLL** が位相追従性能・定常誤差除去に必要。

---



## 2 次 PLL とは？

**Phase-Locked Loop (PLL, 位相ロックループ)** は、外部の入力信号と自分の発振器の **位相** を常に一致させる制御ループです。  
ラジオのチューニング、CD/DVD プレーヤのデータ読取り、各種通信機器のクロック同期など、身近な機器で幅広く使われています。

目的を一言でいえば **「入力信号の位相に自分の位相をぴったり合わせ続けること」**。  
車のナビが現在地と目的地の差を見て進路を修正し続けるのと同じイメージで、PLL は「位相差 (誤差)」を観測し、それをゼロに保つよう発振器の周波数を微調整します。

---

### なぜ **2 個** の積分器が必要なのか？

PLL で位相を正確に追従するには、**2 段の積分器** が必要になります。  
動作は次の 2 段階プロセスで捉えると分かりやすいです。

| 段階 | どこにある積分器か | 積分する対象 | 得られる量 | 役割 |
|------|-------------------|--------------|-----------|------|
| **①** | ループフィルタ（PI）内部 | 位相誤差 *e* | 周波数補正 Δω | 入力の周波数オフセット／ドリフトを吸収し、定常位相誤差をゼロに保つ |
| **②** | VCO/NCO（発振器）内部 | 周波数 Δω | 位相 θ̂ | 指令された周波数から実際の **推定位相** を生成 |

> 位相 → **(1 積分)** → 周波数補正 → **(もう 1 積分)** → 位相  
>  
> これが 2 次 PLL と呼ばれるゆえんです。

---

### `int_err` から $\hat{\omega}_e$ を計算する理由

1. **位相比較器**が検出するのは瞬時の位相誤差 `err`。  
2. **PI ループフィルタ**では  
   \[
   I_k = I_{k-1} + e_k\,T_s \quad(\text{コード中では `int_err`})
   \]  
   として **位相誤差の時間積分**を保持します。
3. その積分値 `int_err` に **積分ゲイン $K_i$** を掛け、さらに現在の誤差 `err` に **比例ゲイン $K_p$** を掛けて合算すると  
   \[
   \hat{\omega}_{e,k} = K_p\,e_k \;+\; K_i\,I_k
   \]  
   という **推定角速度**（VCO/NCO への制御入力）が得られます。
4. **VCO/NCO** が $\hat{\omega}_e$ を **もう一度積分** することで  
   \[
     \hat{\theta}_{e,k} = \hat{\theta}_{e,k-1} + \hat{\omega}_{e,k}\,T_s
   \]  
   **最終的な位相推定値** $\hat{\theta}_e$ が生成されます。

`int_err` を直接 θ̂ に使わないのは、**積分器が 1 段しか無い 1 次 PLL** になってしまい、定常誤差や周波数ドリフトに弱くなるためです。

---

## まとめ

- **2 次 PLL** は「位相誤差 → 周波数補正 → 位相生成」という 2 段階制御で動作する。  
- **2 つの積分器** が必要な理由  
  1. 位相誤差を時間積分して周波数補正を得る（PI フィルタ内）  
  2. 周波数補正を時間積分して推定位相を得る（VCO/NCO 内）
- `int_err` は **位相誤差の積分値**。これにゲインを掛け、現行誤差と組み合わせて **推定角速度 $\hat{\omega}_e$** を生成し、その角速度をさらに積分して **推定位相 $\hat{\theta}_e$** を作る。

> **キーワード:** 2 次 PLL / PI ループフィルタ / VCO (NCO) / 位相誤差 / 周波数補正



