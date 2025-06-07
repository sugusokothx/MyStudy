# dq-axis Motor Model (continuous-time formulation)

Let  

* **inputs**  
  * ${\boldsymbol{v}_{dq}= [\,v_d,\;v_q\,]^{\mathsf T}}$ … stator voltage ( d/q  axes)  
  * ${\omega}$ … electrical angular speed of the rotor  

* **states**  
  * ${ \boldsymbol{\psi}_{dq}= [\,\psi_d,\;\psi_q\,]^{\mathsf T}}$ … stator flux linkages  

* **outputs**  
  * ${ \boldsymbol{i}_{dq}= [\,i_d,\;i_q\,]^{\mathsf T}}$ … phase currents  
  * ${ T_e }$ … electromagnetic torque  

where the currents are obtained from nonlinear magnetisation look-up tables  
(ID MAP & IQ MAP).

---

## 1. Flux–voltage dynamics  

The flux derivative is the algebraic sum of the applied voltage, the resistive drop,  
and the back-EMF term produced by rotation (90 ° rotation matrix ${ \mathbf{J} }$):

$$
\boxed{\;
\dot{\boldsymbol{\psi}}_{dq}
  = \boldsymbol{v}_{dq}
  \;-\; R_s\,\boldsymbol{i}_{dq}
  \;+\; \omega\,\mathbf{J}\,\boldsymbol{\psi}_{dq}
\;}
\qquad
\mathbf{J}
= \begin{bmatrix} 0 & -1 \\[2pt] 1 & 0 \end{bmatrix}.
$$

---

## 2. Magnetisation (lookup inverse)  

From the instantaneous fluxes the currents are read from two independent *n-D* tables  
that capture cross-saturation:

$$
\boldsymbol{i}_{dq}
=
\begin{bmatrix}
i_d \\ i_q
\end{bmatrix}
=
\begin{bmatrix}
f_d\!\bigl(\psi_d,\psi_q,\lVert\boldsymbol{\psi}_{dq}\rVert\bigr) \\[4pt]
f_q\!\bigl(\psi_d,\psi_q,\lVert\boldsymbol{\psi}_{dq}\rVert\bigr)
\end{bmatrix},
\qquad
\lVert\boldsymbol{\psi}_{dq}\rVert=\sqrt{\psi_d^{2}+\psi_q^{2}}.
$$

*`ID MAP ` returns \(f_d\),  *`IQ MAP` returns \(f_q\).*  

---

## 3. Electromagnetic torque  

The Simulink path rotates **i** by \(+90^\circ\) and takes the dot product with **ψ**,  
giving the usual cross-product expression

\[
\boxed{\;
T_e
= P_n\bigl(\psi_d\,i_q\;-\;\psi_q\,i_d\bigr)
\;}
\]

where \(P_n\) is the (scaled) pole–pair factor used in the block *Pn*.  

---

## 4. Summary of the complete model  

$$
\begin{aligned}
\dot{\boldsymbol{\psi}}_{dq}
    &= \boldsymbol{v}_{dq} - R_s\,\boldsymbol{i}_{dq}
       + \omega\,\mathbf{J}\,\boldsymbol{\psi}_{dq}, \\[6pt]
\boldsymbol{i}_{dq}
    &= \bigl[f_d(\psi_d,\psi_q,\|\boldsymbol{\psi}\|),\;
        f_q(\psi_d,\psi_q,\|\boldsymbol{\psi}\|)\bigr]^{\mathsf T}, \\[6pt]
T_e &= P_n(\psi_d\,i_q - \psi_q\,i_d).
\end{aligned}
$$

This set of equations is exactly what the shown Simulink diagram implements:  

* the summer–integrator branch realises the first line (flux dynamics),  
* the ID MAP / IQ MAP tables supply the second line (non-linear magnetic model),  
* the rotation–dot–gain chain computes the third line (torque).  
