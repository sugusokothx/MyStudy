### 0. 目的を整理

* **EKF で得た Ld\_est・Lq\_est と ψ\_f\_est** を使い，

  1. **トルク演算**を「実パラメータ」でアップデート
  2. \*\*電流指令（Id\_ref, Iq\_ref）\*\*を最適化してロスを最小化
  3. **推定トルクと指令トルクの誤差**を閉ループ補償する
     ――という 3 層で補償ループを作るのが定番です。下記では具体的な実装手順と選択肢を示します。

---

## 1. 推定パラメータをどう使うか

| 補償ブロック            | 処理周期         | 役割                                                                                  | 実装のポイント                                            |
| ----------------- | ------------ | ----------------------------------------------------------------------------------- | -------------------------------------------------- |
| **①パラメータ推定（EKF）** | 10 kHz～1 kHz | Ld, Lq, ψ\_f, Id, Iq のオンライン推定                                                       | ノイズがあるので 100 Hz 程度で 1–5 ms の LPF を掛けてから下流へ         |
| **②トルクモデル更新**     | 同上           | $T_{e,\text{est}}=\tfrac{3}{2}P_p\bigl(ψ_f I_q + (L_d-L_q)I_dI_q\bigr)$ を最新パラメータで計算 | 数式は温度や飽和で変わる係数を自動追従 ([eprints.whiterose.ac.uk][1]) |
| **③電流指令生成**       | 1 kHz～10 kHz | a) MTPA/MTPV を再計算<br>b) フィードフォワード補正                                                 | MTPA を解析式で解く or LUT をスケーリング                        |
| **④トルク補償ループ**     | 1 kHz～10 kHz | $ΔT = T^\* - T_{e,\text{est}}$ から **Iq\_ref ← Iq\_ref + k\_T ΔT**                   | k\_T≃(2/3)/(P\_p ψ\_f) 付近で可変                       |

---

## 2. 電流指令の作り方 ― 3 つの代表手法

### A. 解析式 MTPA／MTPV 置き換え

推定した Ld, Lq をそのまま式に代入して Id\_ref, Iq\_ref を計算するやり方です。

$$
\begin{aligned}
T^\* &= \frac{3}{2}P_p\!\bigl(ψ_f I_{q} + (L_d-L_q)I_{d}I_{q}\bigr),\\
\text{minimize }&I_{\text{rms}}=\sqrt{I_d^2+I_q^2}.
\end{aligned}
$$

解析的に解くと

$$
I_{d,\text{MTPA}} = \frac{-ψ_f + \sqrt{ψ_f^{2} + 8 (L_d-L_q) L_q\,T^\*/(1.5P_p)}}{4 (L_d-L_q)}.
$$

Iq\_ref はこの Id\_ref をトルク式に戻して求めます。推定値がそのまま入るので温度・磁気飽和にも追従。

*長所* : 実装が軽い
*短所* : (Ld−Lq) が小さい領域で数値不安定になりやすい

### B. LUT スケーリング

製造時に作った **MTPA LUT** を

$$
\text{scale} = \frac{L_{d,\text{nom}}-L_{q,\text{nom}}}{L_{d,\text{est}}-L_{q,\text{est}}}
$$

で伸縮し，ψ\_f の低下分はトルク係数で補正。既存ファームへの追加が最小です。

### C. オンライン探索型 MTPA

EKF で得た誘導パラメータを初期値にし，**パワー・パートベーション**や **r–θ 法**で Id を微小に揺らして電流総和が増える向きを判定 → 追従。推定パラメータは探索範囲を狭めて応答を高速化する“ガイド”として使います。最近は **正方波電圧注入＋動的 Ld,Lq 推定**を重ねて 200 %以上の定格トルクまで MTPA を追従した例が報告されています ([vicenteyoo.github.io][2])。

---

## 3. トルク補償の実装例フロー

```text
(1)  EKF         :  Ld_est, Lq_est, ψf_est, Id, Iq
(2)  Low-pass    :  τ = 2–5 ms で平滑
(3)  Te_est      :  1.5*Pp*(ψf*Iq + (Ld-Lq)*Id*Iq)
(4)  Id/Iq_ref   :  解析式 MTPA もしくは LUT スケーリング
(5)  ΔT loop     :  Iq_ref ← Iq_ref + kT*(T* – Te_est)
(6)  FOC Current :  PI → SVPWM
```

* **kT** はシンプルに定数でも良いですが，ψ\_f\_est で正規化すればゲイン調整が一発で済みます。
* Flux-weakening領域では **V\_max** 制約に Lq\_est を使うと電圧リミット予測が精度向上 ([ti.com][3])。

---

## 4. 典型的な落とし穴とヒント

| 症状             | 原因                      | 対策                                   |
| -------------- | ----------------------- | ------------------------------------ |
| 推定 Ld,Lq がギザギザ | 電流センサ量子化・デッドタイム誤差       | EKF の R 行列を速度依存で可変に／ハイレゾ AD 占有       |
| トルク補償が発散       | ψ\_f\_est の過小評価で kT が過大 | ψ\_f\_est を 1 pu 固定にリミット or kT を適応制御 |
| 低速域で MTPA が不安定 | 解析式で Ld≈Lq となり Id が発散   | LUT スケーリング or 探索法に切替え                |

---

## 5. さらに精度を上げたいときは…

1. **非線形磁気モデル**
   \- $L_{dq}(i_d,i_q)$ サチュレーション写像を 2D 多項式で持ち，EKF は ΔLd, ΔLq のみ推定。
2. **MPC に埋め込み**
   \- コスト関数に $|Te^\* - Te_{\text{est}}|$ を直接入れると Id/Iq を自動最適化。
3. **サーマルモデルと結合**
   \- Ld,Lq 温度係数を使って推定値→巻線温度を推定し，過熱保護にも活用。

---

### 主要参考文献

* Zhu et al., *Online Parameter Estimation for PMSMs: An Overview*, IEEE Access 2021 ([eprints.whiterose.ac.uk][1])
* Yoo et al., *Online MTPA Tracking with Dynamic Inductance Estimator*, T-PEL 2022 ([vicenteyoo.github.io][2])
* TI Application Report *Sensorless-FOC With Flux-Weakening and MTPA*, SPRACF3 2018 ([ti.com][3])
* Recent EKF-based adaptive torque control approach, *Non-interacting Optimal and Adaptive Torque Control*, IFAC 2024 ([sciencedirect.com][4])

---

## まとめ

1. **推定パラメータ → トルクモデル更新 → 電流指令補正**の 3 層で構成。
2. MTPA/MTPV の生成は (解析式 / LUTスケール / 探索法) から選択。
3. フィードバックで **ΔT** を閉ループにすると線形化が容易。

この枠組みなら，既存 FOC へ最小限の変更で **温度・飽和変動に強いトルク制御**が実現できます。

[1]: https://eprints.whiterose.ac.uk/id/eprint/173931/1/09402773.pdf "Online parameter estimation for permanent magnet synchronous machines : an overview"
[2]: https://vicenteyoo.github.io/assets/papers/TPE2022_1_AcceptedVersion.pdf "Microsoft Word - TPE_Manuscript_EA"
[3]: https://www.ti.com/lit/pdf/spracf3 "Sensorless-FOC With Flux-Weakening and MTPA Motor Drives"
[4]: https://www.sciencedirect.com/science/article/pii/S0019057824006244?utm_source=chatgpt.com "Noninteracting optimal and adaptive torque control using an online ..."
