### 拡張カルマンフィルタ（エスケープ文字を追加）

| カテゴリ | 式 |
|:---|:---|
| **予測ステップ** | |
| 状態予測 | $ \hat{x}_{k\|k-1} = \hat{x}_{k-1\|k-1} + f(\hat{x}_{k-1\|k-1}, u_{k-1}) \Delta t $ |
| ヤコビ行列 | $ F_k = \frac{\partial f}{\partial x} \bigg|_{\hat{x}_{k-1\|k-1}, u_{k-1}} $ |
| 誤差共分散予測 | $ P_{k\|k-1} = (I + F_k \Delta t) P_{k-1\|k-1} (I + F_k \Delta t)^\top + Q $ |
| **更新ステップ** | |
| イノベーション共分散 | $ S_k = H P_{k\|k-1} H^\top + R $ |
| カルマンゲイン | $ K_k = P_{k\|k-1} H^\top S_k^{-1} $ |
| 状態更新 | $ \hat{x}_{k\|k} = \hat{x}_{k\|k-1} + K_k (z_k - H \hat{x}_{k\|k-1}) $ |
| 誤差共分散更新 | $ P_{k\|k} = (I - K_k H) P_{k\|k-1} $ |


| ステップ | 式 | 実装 |
|---------|------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| 予測    | $\hat x_{k\mid k-1}= \hat x_{k-1}+f(\hat x_{k-1},u_{k-1})\Delta t$                                                                | オイラー前進 |
|         | $F_k=\left.\dfrac{\partial f}{\partial x}\right\rvert_{\hat x_{k-1},u_{k-1}}$                                                     | 前進差分 (`numerical_jacobian`) |
|         | $P_{k\mid k-1}=(I+F_k\Delta t)P_{k-1}(I+F_k\Delta t)^\top+Q$                                                                     |               |
| 更新    | $S_k=HP_{k\mid k-1}H^\top+R$                                                                                                      |               |
|         | $K_k=P_{k\mid k-1}H^\top S_k^{-1}$                                                                                                |               |
|         | $\hat x_{k}= \hat x_{k\mid k-1}+K_k\bigl(z_k-H\hat x_{k\mid k-1}\bigr)$                                                           |               |
|         | $P_k=(I-K_k H)P_{k\mid k-1}$                                                                                                      |               |
