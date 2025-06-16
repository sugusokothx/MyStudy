# 回転ベクトル同伴フィルタ (VRAF)

ソフトウェア要求定義書

**版数**: 1.0\
**作成日**: 2025‑06‑16\
**作成者**: ChatGPT

---

## 1. 目的

本書は、三相電力システムやモータ制御で発生する \(\alpha\beta\) 二相信号から特定の周波数成分（正・負の回転成分）を抽出する**回転ベクトル同伴フィルタ (Vector‑Rotator‑Assisted Filter: VRAF)** のソフトウェア実装に関する要求事項を定義するものである。実装対象コードは Python クラス `VRAF` とし、リアルタイム処理またはオフライン解析での利用を想定する。

## 2. 背景と原理

### 2.1 信号モデル

二相入力電圧（または電流）を実部・虚部で構成する複素信号

$$
  u_p(t) = u_\alpha(t) + j\,u_\beta(t)\quad (1)
$$

を考える。ここで \(j=\sqrt{-1}\)。抽出したい成分の角周波数を \(\omega = 2\pi f_{\text{shift}}\) とする。

### 2.2 回転座標変換（デモジュレーション）

| 抽出対象                     | 変換式                                                     |
| ------------------------ | ------------------------------------------------------- |
| 正相成分 (positive‑sequence) | \(\tilde u(t)=u_p(t)\,e^{-j\omega t}\)  \(\text{(2a)}\) |
| 逆相成分 (negative‑sequence) | \(\tilde u(t)=u_p(t)\,e^{+j\omega t}\)  \(\text{(2b)}\) |

デモジュレーションにより抽出周波数成分は直流 (DC) 成分へ変換され、不要な成分は \(\pm 2\omega\) 以上の高周波へ移動する。

### 2.3 IIR ローパスフィルタ (LPF)

デモジュレーション後の複素信号 \(\tilde u(t) = \Re\{\tilde u(t)\} + j\Im\{\tilde u(t)\}\) に対し、実部・虚部をそれぞれ **デジタル Butterworth フィルタ**で低域通過させる。

*離散時間*の伝達関数は

$$
H(z)=\frac{b_0+b_1 z^{-1}+\dots+b_M z^{-M}}{1+a_1 z^{-1}+\dots+a_N z^{-N}}\quad (3)
$$

で与えられ、設計パラメータは次数 \(N=\text{order}\)（偶数を推奨）とカットオフ周波数 \(f_c\)。実装では SciPy の `butter()` 関数を用いて係数を生成し、`lfilter()` で状態空間を保持しながら逐次処理する。

### 2.4 リモジュレーション（回転戻し）

LPF 出力 \(\tilde x(t)=x_R(t)+j x_I(t)\) を、デモジュレーションと逆符号で再回転させる。

| 抽出対象 | 変換式                                                     |
| ---- | ------------------------------------------------------- |
| 正相成分 | \(y_p(t)=\tilde x(t)\,e^{+j\omega t}\)  \(\text{(4a)}\) |
| 逆相成分 | \(y_p(t)=\tilde x(t)\,e^{-j\omega t}\)  \(\text{(4b)}\) |

最終的な出力は実部と虚部を取り出して \( y_\alpha(t)=\Re\{y_p(t)\},\qquad y_\beta(t)=\Im\{y_p(t)\}\tag{5}\) とする。

---

## 3. 用語定義

| 略語                   | 意味                             |
| -------------------- | ------------------------------ |
| VRAF                 | Vector‑Rotator‑Assisted Filter |
| LPF                  | Low‑Pass Filter（低域通過フィルタ）      |
| 正相 / 逆相              | 電力系統の対称分解による正順・逆順成分            |
| \(f_{\text{shift}}\) | 抽出対象となる高調波周波数 [Hz]             |

---

## 4. 機能要求

| ID        | 要求                                                             | 優先度 |
| --------- | -------------------------------------------------------------- | --- |
| **FR‑01** | インスタンス生成時に `f_shift`, `Fs`, `seq`, `fc`, `order` を指定できること      | 高   |
| **FR‑02** | `process(u_α, u_β)` 呼び出しごとに 1 サンプルを入力し、抽出後の \(y_α, y_β\) を返すこと | 高   |
| **FR‑03** | 内部状態（フィルタ遅延・サンプルカウンタ）を `reset()` で初期化可能であること                   | 中   |
| **FR‑04** | 正相 (`seq='pos'`) と逆相 (`seq='neg'`) の両モードをサポートすること              | 高   |
| **FR‑05** | スカラー演算のみで実時間サンプリングレート \(F_s\le 100\,\text{kHz}\) に対応すること       | 低   |

---

## 5. 非機能要求

| ID         | 要求  | 内容                                                             |
| ---------- | --- | -------------------------------------------------------------- |
| **NFR‑01** | 性能  | 1 サンプルあたり計算遅延 \(<4\,\mu\text{s}\)（Python 3.12, 3.2 GHz CPU 相当） |
| **NFR‑02** | 精度  | 目的周波数での振幅誤差 \(<1\,\%)、位相誤差 \(<0.5°) \)                         |
| **NFR‑03** | 可搬性 | NumPy, SciPy 以外の外部依存を持たないこと                                    |
| **NFR‑04** | テスト | ユニットテスト (pytest) を 95 % 以上の分岐網羅率で提供すること                        |

---

## 6. インタフェース仕様

```python
class VRAF:
    def __init__(self,
                 f_shift: float,
                 Fs: float,
                 seq: str = 'pos',
                 fc: float = 150,
                 order: int = 4):
        """__init__ 参照"""

    def reset(self) -> None:
        """内部状態をクリア"""

    def process(self, u_alpha: float, u_beta: float) -> tuple[float, float]:
        """1 サンプル処理し (y_alpha, y_beta) を返す"""
```

引数・戻り値はすべて Python `float` とする。`seq` は `'pos'` または `'neg'`。

---

## 7. エラーハンドリング

- 不正パラメータ（負の周波数、偶数でない `order`, `seq` 値の不一致）に対しては `ValueError` を送出すること。
- サンプリング周波数がナイキスト条件を満たさない場合には `RuntimeError` を送出すること。

---

## 8. テスト要求

1. **周波数応答テスト**
   - 入力: 単一周波数信号 \(f=f_{\text{shift}}\pm\varepsilon\)
   - 期待: 通過帯域での振幅誤差 \(<1\,\%\)。
2. **シーケンス分離テスト**
   - 入力: 同振幅の正相・逆相信号を重畳
   - 期待: 指定シーケンスのみ抽出、他方は \(<-40\,\text{dB}\)。
3. **リセット試験**
   - `reset()` 後に出力が 0 付近に戻ること。

---

## 9. 制約事項

- Butterworth 次数は**偶数推奨**。偶数次数ではフィルタの一次遅延成分が偶要素で対称になり、正相・逆相の両チャネルで同位相応答が確保しやすい。奇数次数の場合、実部・虚部間で位相歪みが発生し、抽出精度が劣化するおそれがある。
- 回転演算は CORDIC など固定小数点化可能だが、本実装では浮動小数点とする。

---

## 10. 参考文献

1. F. Blaabjerg *et al.* "Power Electronics – Devices, Drivers, Applications, and Reliability." 2021.
2. IEEE Std 1159‑2019 "IEEE Recommended Practice for Monitoring Electric Power Quality."
3. SciPy `signal` モジュール API リファレンス.

---

## 付録 A. 使用例

```python
import numpy as np
from scipy.signal import chirp
from vraf import VRAF

Fs = 10_000  # 10 kHz
f_h = 1_000  # 1 kHz 高調波
vraf = VRAF(f_shift=f_h, Fs=Fs, seq='pos', fc=150, order=4)

t = np.arange(0, 0.5, 1/Fs)
# 正相 1 kHz + 雑音 200 Hz
u_alpha = np.cos(2*np.pi*200*t) + 0.7*np.cos(2*np.pi*f_h*t)
u_beta  = np.sin(2*np.pi*200*t) + 0.7*np.sin(2*np.pi*f_h*t)

y_alpha = np.empty_like(t)
y_beta  = np.empty_like(t)
for k in range(len(t)):
    y_alpha[k], y_beta[k] = vraf.process(u_alpha[k], u_beta[k])
```

