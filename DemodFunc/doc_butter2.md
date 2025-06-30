
```python
import numpy as np
from math import pi, tan, sin
from typing import Tuple


# ------------------------------------------------------------
# 1) helper – analogue-to-digital biquad coefficient converter
# ------------------------------------------------------------
def _biquad_coeff(omega_c: float, fs: float, zeta: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Return *normalised* (a0 = 1) digital biquad (b, a) for one
    analogue 2-nd-order Butterworth section, bilinear-transformed.
    """
    T  = 1.0 / fs
    K  = 2.0 / T            # 2 * f_s
    A0 = K * K
    A1 = 2.0 * zeta * omega_c * K
    A2 = omega_c * omega_c

    # digital denom (before normalise)
    a0 = A0 + A1 + A2
    a1 = 2.0 * (A2 - A0)
    a2 = A0 - A1 + A2

    # digital numer
    b0 = A2
    b1 = 2.0 * A2
    b2 = A2

    b = np.array([b0, b1, b2]) / a0
    a = np.array([1.0, a1 / a0, a2 / a0])
    return b, a


# ------------------------------------------------------------
# 2)   design + real-time class : 2-nd-order Butterworth LPF
# ------------------------------------------------------------
class ButterworthLPF2:
    """
    **Real-time 2-nd-order Butterworth LPF**

    • Direct-Form-II transposed biquad (single section)  
    • Same public API as ButterworthLPF4:  reset(), process_sample(), process_array()
    """
    def __init__(self, fc: float, fs: float):
        omega_c = 2.0 * fs * tan(pi * fc / fs)          # pre-warp
        zeta    = sin(pi / 4)                           # ζ = √2/2 for N=2
        b, a    = _biquad_coeff(omega_c, fs, zeta)
        self.b0, self.b1, self.b2 = b
        _, self.a1, self.a2 = a
        self.z = np.zeros(2)                            # DF-II T states

    # -----------------------------------------------------
    def reset(self):
        self.z[:] = 0.0

    def process_sample(self, x: float) -> float:
        """Filter one sample (float-in / float-out)."""
        y       = self.b0 * x + self.z[0]
        self.z[0] = self.b1 * x + self.z[1] - self.a1 * y
        self.z[1] = self.b2 * x              - self.a2 * y
        return y

    def process_array(self, x: np.ndarray) -> np.ndarray:
        """Vectorised convenience wrapper."""
        y = np.empty_like(x, dtype=float)
        for n, val in enumerate(x):
            y[n] = self.process_sample(float(val))
        return y
```

### 使い方例

```python
Fs = 12_000          # 12 kHz
fc = 250.0           # 250 Hz cut-off

lpf2 = ButterworthLPF2(fc, Fs)

y = lpf2.process_sample(x0)         # 1 サンプル処理
y_vec = lpf2.process_array(x_vec)   # ベクトル一括
```

*係数は `numpy` と `math` だけで生成でき、SciPy 依存はありません。
4 次版より群遅延が約半分になる代わりに、高調波抑圧は 12 dB ほど低下します。*
