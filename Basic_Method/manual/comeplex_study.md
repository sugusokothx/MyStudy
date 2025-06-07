記号定義（LPF 後の定常値）

$i^{(s)}_{\alpha} \triangleq \frac12 \hat I_{h,\alpha}\cos\varphi_{\alpha}$

$i^{(c)}_{\alpha} \triangleq \frac12 \hat I_{h,\alpha}\sin\varphi_{\alpha}$

$i^{(s)}_{\beta } \triangleq \frac12 \hat I_{h,\beta }\cos\varphi_{\beta }$

$i^{(c)}_{\beta } \triangleq \frac12 \hat I_{h,\beta }\sin\varphi_{\beta }$

$\hat I_{h,(\cdot)}$ 

$\varphi_{(\cdot)}$

$ I_{h,\alpha}
     = 2\bigl(i_{\alpha}^{(s)} + j\,i_{\alpha}^{(c)}\bigr)
     = 2\Bigl(\tfrac12\,\hat I_{h,\alpha}\cos\varphi_{\alpha}
            + j\,\tfrac12\,\hat I_{h,\alpha}\sin\varphi_{\alpha}\Bigr)
     = \hat I_{h,\alpha}\bigl(\cos\varphi_{\alpha} + j\sin\varphi_{\alpha}\bigr)
     = \hat I_{h,\alpha}\,e^{j\varphi_{\alpha}} $

$ I_{h,\beta} = \hat I_{h,\beta}\,e^{j\varphi_{\beta}} $


$ \mathbf I_{h,\alpha\beta}
   = \begin{bmatrix}
       I_{h,\alpha} \\
       I_{h,\beta}
     \end{bmatrix}
   = \begin{bmatrix}
       \hat I_{h,\alpha}\,e^{j\varphi_{\alpha}} \\
       \hat I_{h,\beta}\,e^{j\varphi_{\beta}}
     \end{bmatrix} $

$ \operatorname{Re}\{\mathbf I_{h,\alpha\beta}\}
   = \begin{bmatrix}
       \hat I_{h,\alpha}\cos\varphi_{\alpha} \\
       \hat I_{h,\beta}\cos\varphi_{\beta}
     \end{bmatrix} $



$2
\begin{bmatrix}
  i^{(s)}_{\alpha}\\
  i^{(s)}_{\beta}
\end{bmatrix}
=
\begin{bmatrix}
  \hat I_{h,\alpha}\cos\varphi_{\alpha}\\
  \hat I_{h,\beta}\cos\varphi_{\beta}
\end{bmatrix}$


**まとめ**

- `×2` で LPF による $\tfrac12$ ゲインを打ち消し、  
- `real` を取ることで **I チャネル（cos 成分）**だけを抽出している。  
  位相ずれ $\varphi$ の情報は虚部（Q チャネル）に残り、実部は純粋に “コサイン側のゲイン付き振幅” になります。



