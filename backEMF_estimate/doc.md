下記は **Δφ 方式** をそのまま実装できる “たたき台” です。

* **状態ベクトル** `x = [ i_d, i_q, dphi_d, dphi_q, R_s ]ᵀ`
* **観測** `z = [ i_d_obs, i_q_obs ]ᵀ`
* **入力** `u = [ v_d, v_q, omega_e ]ᵀ`
* **磁束マップ** あらかじめ実機計測データを

  ```python
  # 例）len(IdAxis)=81, len(IqAxis)=81
  IdAxis = np.linspace(-300, 300, 81)   # [A]
  IqAxis = np.linspace(-300, 300, 81)
  PhiDMap = ...   # shape (81,81)  φd [Wb]
  PhiQMap = ...   # shape (81,81)  φq [Wb]
  ```

  としてロードしておく想定です。

> **ポイント**
>
> * Δφ の時間変化は **ランダムウォーク**（`0 + w`）
> * 電流ダイナミクスは「線形 Ld/Lq モデル」で予測しておくと安定
>   （マップの微分から逆行列 *M⁻¹* を組むのは後で改良すれば OK）

```python
import numpy as np
from scipy.interpolate import RegularGridInterpolator

# ────────────────────────────────────────────────────────────
# 0)  磁束マップ LUT 準備
# ────────────────────────────────────────────────────────────
IdAxis = ...                      # 1-D array  (実機データで置き換え)
IqAxis = ...
PhiDMap = ...                     # shape (len(IdAxis), len(IqAxis))
PhiQMap = ...

phi_d_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiDMap,
                                    bounds_error=False, fill_value=None)
phi_q_lut = RegularGridInterpolator((IdAxis, IqAxis), PhiQMap,
                                    bounds_error=False, fill_value=None)

# ────────────────────────────────────────────────────────────
# 1)  ユーティリティ
# ────────────────────────────────────────────────────────────
def numerical_jacobian(f, x, u, eps=1e-6):
    """f(x,u) の ∂f/∂x を前進差分で求める"""
    n = len(x)
    Fx = np.zeros((n, n))
    f0 = f(x, u)
    for k in range(n):
        x1 = x.copy()
        x1[k] += eps
        Fx[:, k] = (f(x1, u) - f0) / eps
    return Fx

# ────────────────────────────────────────────────────────────
# 2)  Δφ-EKF 本体
# ────────────────────────────────────────────────────────────
class DeltaPhiEKF:
    """
    状態:  x = [ i_d, i_q, dphi_d, dphi_q, R_s ]^T
    観測:  z = [ i_d_obs, i_q_obs ]^T
    """
    def __init__(self, dt, Ld, Lq,
                 Q_diag=(1e-6, 1e-6, 1e-8, 1e-8, 1e-9),
                 R_diag=(1e-4, 1e-4)):
        self.dt = dt
        self.Ld, self.Lq = Ld, Lq

        # 推定値初期化
        self.x_hat = np.zeros((5, 1))
        self.P = np.diag([1e-3, 1e-3, 1e-4, 1e-4, 1e-4])

        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0]])

        # --- 内部キャッシュ ---
        self._psi_f_nom = PhiQMap[0, 0]   # 例: I=0 の q-軸値を名目磁石鎖交磁束とみなす

    # --------------- 予測 -----------------------------------
    def _f_continuous(self, x, u):
        """連続時間の状態方程式 f(x,u)  (dx/dt)"""
        i_d, i_q, dphi_d, dphi_q, R_s = x.flatten()
        v_d, v_q, w_e = u

        # マップ + Δφ
        phi_d = float(phi_d_lut([i_d, i_q])) + dphi_d
        phi_q = float(phi_q_lut([i_d, i_q])) + dphi_q

        # 電流ダイナミクス (線形仮定)
        di_d = ( -R_s * i_d + w_e * self.Lq * i_q + v_d ) / self.Ld
        di_q = ( -R_s * i_q - w_e * self.Ld * i_d
                 - w_e * self._psi_f_nom + v_q ) / self.Lq

        # Δφ と R_s はランダムウォーク
        return np.array([di_d, di_q, 0.0, 0.0, 0.0])

    def predict(self, u):
        # dx/dt & オイラー離散化
        f_val = self._f_continuous(self.x_hat.flatten(), u)
        self.x_hat = self.x_hat + f_val.reshape(-1, 1) * self.dt

        # ヤコビアン F ≈ ∂f/∂x
        F = numerical_jacobian(self._f_continuous,
                               self.x_hat.flatten(), u)

        F_k = np.eye(5) + F * self.dt
        self.P = F_k @ self.P @ F_k.T + self.Q

    # --------------- 更新 -----------------------------------
    def update(self, z):
        z = z.reshape(2, 1)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x_hat
        self.x_hat += K @ y
        self.P = (np.eye(5) - K @ self.H) @ self.P

    # --------------- アクセサ -------------------------------
    @property
    def i_d(self):      return self.x_hat[0, 0]
    @property
    def i_q(self):      return self.x_hat[1, 0]
    @property
    def phi_d(self):
        return float(phi_d_lut([self.i_d, self.i_q])) + self.x_hat[2, 0]
    @property
    def phi_q(self):
        return float(phi_q_lut([self.i_d, self.i_q])) + self.x_hat[3, 0]
    @property
    def torque(self, pole_pairs=4):
        return 1.5 * pole_pairs * (self.phi_d * self.i_q -
                                   self.phi_q * self.i_d)

# ────────────────────────────────────────────────────────────
# 3)  使いかたサンプル
# ────────────────────────────────────────────────────────────
if __name__ == "__main__":

    DT = 1e-4
    Ld, Lq = 0.3e-3, 0.5e-3           # [H] ざっくり値
    ekf = DeltaPhiEKF(DT, Ld, Lq)

    # ── 簡易シミュレーション：名目電圧を印可 ───────────────
    v_d_cmd, v_q_cmd = 30.0, 80.0     # [V]
    omega_e = 2*np.pi*200             # 200 Hz

    for k in range(20000):
        u = (v_d_cmd, v_q_cmd, omega_e)
        ekf.predict(u)

        # 真値として Ld/Lq モデルで電流を生成 (ここは実機置き換え)
        # 今回は単に推定値に小ノイズを足して「疑似センサ値」に
        z = np.array([ekf.i_d, ekf.i_q]) + \
            np.random.normal(0, np.sqrt(1e-4), 2)

        ekf.update(z)

        if k % 1000 == 0:
            print(f"k={k:5d},  φd={ekf.phi_d:+.3f}  φq={ekf.phi_q:+.3f}  "
                  f"Te={ekf.torque:+.2f} Nm")

```

### 使い始める際のチェックリスト

1. **マップデータを実機値に置換**
   `IdAxis`, `IqAxis`, `PhiDMap`, `PhiQMap` をそのまま測定テーブルに。
2. **ψ<sub>f</sub> 名目値**
   `self._psi_f_nom` は `(Id, Iq) = (0,0)` の φ<sub>q</sub> を採用しましたが
   「0 電流でも q-軸磁束 ≈ 磁石鎖交磁束」というモータ前提が違えば変更してください。
3. **ノイズ共分散 Q/R**
   行列対角要素を 10×↔0.1× でスイープし、収束速度と振動を確認。
4. **ヤコビアンの負荷**
   `numerical_jacobian` は 5 回 `f()` を呼び出すだけなので
   100 µs ループでも Cortex-M7 で 10 µs 程度。
   さらに軽量化したい場合は解析式に置き換え ─ ただしバグ率が上がるので後回し推奨。

---

この skeleton から **Δφ 方式 EKF** をすぐ試せます。
次に

* 実測波形をリプレイして推定品質を可視化
* トルク補償ループへ組み込んでベンチ試験

…に進むとき、追加のコードや数式が必要になったらまたご相談ください。
