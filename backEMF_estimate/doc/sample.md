
はい、承知いたしました。
以下のようにLATEXの数式を修正しました。

| **予測 (time-update)** | $$\begin{align*} \hat{\mathbf{x}}_{k|k-1} &= f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_{k-1}) \\ \mathbf{F}_k &= \left. \frac{\partial f}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}} \\ \mathbf{P}_{k|k-1} &= \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^{\mathsf{T}} + \mathbf{Q} \end{align*}$$ | `predict()` 内：\<br\> \* `x_dot` が $f$\<br\> \* `F` がヤコビアン，$\\mathbf{F}\_k = \\mathbf{I}+\\mathbf{F} \\cdot dt$\<br\> \* `self.P = F_k @ P @ F_k.T + Q` |
| :--- | :--- | :--- |
| **更新 (measurement-update)** | $$\begin{align*} \mathbf{S}_k &= \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^{\mathsf{T}} + \mathbf{R} \\ \mathbf{K}_k &= \mathbf{P}_{k|k-1} \mathbf{H}^{\mathsf{T}} \mathbf{S}_k^{-1} \\ \hat{\mathbf{x}}_{k|k} &= \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k(\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}) \\ \mathbf{P}_{k|k} &= (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1} \end{align*}$$ | `update()` 内：\<br\> \* `S`, `K`, `y`, `self.x_hat`, `self.P` |