| 観点            | Pref 補正方式 (A)                   | VdVq 補正方式 (B)           |
| ------------- | ------------------------------- | ----------------------- |
| **改修規模**      | Pref テーブルを温度軸付きにするだけ（スカラー）      | Vd,Vq を毎サイクル補正（ベクトル演算）  |
| **必要な入力**     | Rs(T) か ΔRs(T) と                | 同左                      |
| **演算負荷**      | 低                               | やや高 (×Id,×Iq)           |
| **診断の分かりやすさ** | Pref が温度で動くのでロジックが見えやすい         | P\_meas が直接温度追従、Pref 不変 |
| **危険な二重補正**   | Pref と VdVq の両方で ΔRs を考慮しないよう注意 | 同左                      |


| アプローチ            | 何を補正するか                | P の計算式 (実装例)                                                                                 | Pref 側で必要な手当て                     | 本質的に入る ΔRs 項 |     |            |   |        |
| ---------------- | ---------------------- | -------------------------------------------------------------------------------------------- | --------------------------------- | ------------ | --- | ---------- | - | ------ |
| **A. Pref を動かす** | **1. Pcu 分を Pref に足す** | `P_meas = Vd_cmd*Id + Vq_cmd*Iq` (指令電圧のまま)                                                   | \`Pref(T) = Pref\_nom + 3·ΔRs(T)· | I            | ²\` | ＋3 ΔRs     | I | ² （銅損） |
| **B. 電圧を動かす**    | **2. VdVq を Rs 落差で補正** | `Vd_eff = Vd_cmd - ΔRs·Id`<br>`Vq_eff = Vq_cmd - ΔRs·Iq`<br>`P_meas = Vd_eff*Id + Vq_eff*Iq` | Pref は **元のまま**で良い                | −3 ΔRs       | I   | ² （電圧側で相殺） |   |        |


https://ietresearch.onlinelibrary.wiley.com/doi/epdf/10.1049/iet-epa.2018.5196?utm_source=chatgpt.com


| #                                                                     | 指標 (制御変数)                                                                                                                                          | 代表的な利用法・論文                       | 効果報告                         | 主なデメリット |
| --------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------- | ---------------------------- | ------- |
| **① 電磁トルク $\hat T_e$**<br>$\tfrac32 p(\hat\phi_d i_q-\hat\phi_q i_d)$ | *DTC/DTFC* や **トルクオブザーバ FB**<br>   ▷ **Océn (2005) DTC 改良版** ([diva-portal.org][1])                                                                | IPMSM 7 kW：リプル −55 %             | φ の高帯域推定が必須；ノイズで∂T/∂t が暴れる   |         |
| **② アクティブフラックス $\psi_a$**<br>$\psi_a ≜ \psi_f+(L_d-L_q)i_d$           | **Active-Flux Control／Sensorless**<br>   ▷ **Boldea et al. (OPTIM 2008)** <br>   ▷ **Li et al. (2022) MT per Active-Flux** ([researchgate.net][2]) | 2 rpm–1000 rpm で半定格トルク保持、位置センサレス | ψa は温度・飽和で変動 → 補償パラメータが増える   |         |
| **③ 磁気（共）エネルギー $W$**<br>$\tfrac12(\phi_d i_d+\phi_q i_q)$             | **共エネルギー最小化／リプル補償**<br>   ▷ **Nakao et al. (2010)** <br>   ▷ **Zhang et al. (2019)** ([ietresearch.onlinelibrary.wiley.com][3])                    | リプル −30〜50 % / 実装簡易              | W=constでも T 定在誤差 ±1–3 % 残存   |         |
| **④ 定トルク曲線上の (id, iq)**                                               | **Constant-Torque Curve Tracking**<br>   ▷ MathWorks constraint-curve実装解説 ([mathworks.com][4])                                                     | 解析式ベースで誤差 <1 % (シミュレーション)        | Ld,Lq,ψf の温度・飽和依存を都度再計算が必要   |         |
| **⑤ MTPA 角 (β=tan⁻¹(i\_d/i\_q))**                                     | **Max-Torque-per-Ampere†** (＝必要電流最小化 → 間接的に T=const を狙う）<br>   ▷ IET Electron. Lett. 2024 MTPA IPMSM ([ietresearch.onlinelibrary.wiley.com][5])    | 温度補償込みで T 誤差 ±0.5 % / I²R −6 %   | β は T に応じて変動するため *指令再演算* が必須 |         |

[1]: https://www.diva-portal.org/smash/get/diva2%3A582454/FULLTEXT01.pdf?utm_source=chatgpt.com "[PDF] Direct Torque Control of a Permanent Magnet synchronous Motor"
[2]: https://www.researchgate.net/publication/376374348_An_optimal_maximum_torque_per_active_flux_and_field_weakening_operation_for_deadbeat_direct_torque_control_based_IPMSM_drive?utm_source=chatgpt.com "An Optimal Maximum Torque Per Active Flux and Field Weakening ..."
[3]: https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/iet-epa.2018.5196?utm_source=chatgpt.com "Research on PMSM harmonic coupling models based on magnetic ..."
[4]: https://www.mathworks.com/help/mcb/gs/pmsm-constraint-curves-and-their-application.html?utm_source=chatgpt.com "PMSM Constraint Curves and Their Application - MathWorks"
[5]: https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/elp2.12453?utm_source=chatgpt.com "Maximum torque per ampere control of permanent magnet ..."
