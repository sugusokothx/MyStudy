ありがとうございます。EKFの状態にRs（巻線抵抗）を追加するアプローチの妥当性、設計上の見通し、ならびにΔRs vs 絶対Rsのどちらを推定すべきかという観点を含めて評価します。

結果をまとめ次第お知らせします。


# Adding Rs to the EKF State for Flux Estimation: Feasibility and Considerations

Including the stator resistance $R_s$ as part of the EKF state vector is a known approach to improve flux estimation under temperature variations. By estimating $R_s$ online, the EKF can distinguish voltage drops caused by rising resistance from those caused by changes in flux linkage. Overall, this extension is **feasible** and can enhance the accuracy of flux estimation, provided the filter is properly tuned and the system remains observable. Below, we address each of your specific points in detail:

## 1. Linear Temperature Characteristic of Rs and Baseline Modeling

It is reasonable to model $R_s$ as linearly dependent on temperature (the copper winding resistance increases roughly 0.4% per °C in the normal operating range). In other words, $R_s(T) \approx R_{s}(T_0)\,[1 + \alpha (T - T_0)]$ with $\alpha$ being the temperature coefficient of resistivity. This linear approximation is usually sufficient for copper over typical motor temperature ranges. In practice, if you have an expected or measured stator temperature, you **can predict $R_s$** from it as a baseline. For example, the stator winding resistance rises with heating due to copper’s resistivity, while the permanent magnet flux decreases as temperature rises. Inductances are relatively temperature-invariant. Knowing this, one strategy is to use a known nominal $R_s$ at a reference temperature (say 20°C) and incorporate the linear model as prior knowledge. This baseline $R_s$ value (or its expected variation) can be **defined in the EKF** either as a fixed initial state or even as a time-varying input if an external temperature estimate is available. In summary, treating $R_s$ with a linear temperature model is appropriate and can be leveraged to improve estimation (either by guiding the EKF or as a check on the EKF’s estimated $R_s$). It aligns with literature recommending on-line adaptation of temperature-dependent parameters either via direct estimation or via known temperature relationships.

## 2. Impact of Rs Variation Speed (Slow vs. Fast Change)

**Assuming Slow Change:** In reality, $R_s$ changes relatively slowly compared to electrical dynamics – the stator winding’s thermal time constant is on the order of seconds or longer, depending on the motor size and cooling. This means during normal operation, $R_s$ will drift gradually as the machine heats or cools. The EKF can treat $R_s$ as a quasi-constant parameter with a slow random walk. In the state-space model, you would add $R_s$ as an additional state (or Δ$R_s$; more on that shortly) that evolves with near-zero dynamics (e.g. $R_{s,k+1} \approx R_{s,k}$ plus process noise). With a small process noise covariance $Q_{R_s}$, the filter will **slowly adjust $R_s$** to account for long-term drifts. This should work well because any persistent voltage prediction error (when currents and speed vary) can be attributed to an $R_s$ deviation and corrected over time. Studies have shown that including stator resistance in the EKF state alongside flux can yield accurate tracking of the true resistance, which in turn keeps flux estimates accurate. In one example, an EKF was able to estimate a stator winding resistance to within a few milliohms of the actual value, and as a result the magnet flux linkage estimate remained precise. This demonstrates that for gradual parameter changes, the EKF can converge and maintain high performance.

**If $R_s$ Changes “Fast”:** Although rapid changes in $R_s$ are not typical (barring a sudden temperature spike or a fault), it’s insightful to consider a scenario of faster variation. If $R_s$ were to change more quickly than expected, the EKF’s ability to track it depends on the process noise tuning and the system’s excitation. With the default assumption of slow change (small $Q_{R_s}$), a fast actual change would initially appear as a modeling error — the filter might momentarily attribute the effect to other states (like Δφ) until the residual induces an $R_s$ update. This could cause a lag in the estimation or a transient bias in flux estimation. To handle faster changes, you would increase the process noise for the $R_s$ state, allowing the filter to **adapt more quickly**. The trade-off is that higher process noise can introduce more estimation jitter or susceptibility to measurement noise (the filter might chase noise by adjusting $R_s$ unnecessarily). In outlook, if you anticipate only slow thermal changes (which is usually the case), you keep $Q_{R_s}$ low for stability. If a scenario arises with quicker resistance changes, the EKF can still track them by tuning up $Q_{R_s}$, but you should then carefully monitor for any interaction where the filter might confuse flux and $R_s$ effects. Fortunately, the effects of $R_s$ and flux on the model are not identical, which aids observability: a change in $R_s$ affects the voltage drop term $R_s i_d, R_s i_q$ in **both** d- and q-axes, whereas a change in magnet flux (Δφ) affects the back-EMF term $ω \phi$ **with opposite signs** in the d vs. q equations. This difference means that with adequate excitation (variations in current and speed), the EKF can distinguish $R_s$ changes from flux changes. In summary, the approach works best for slow-to-moderate $R_s$ drift; it can be tuned for faster changes but with some loss of filtering smoothness. Generally, large or abrupt jumps in $R_s$ are not expected in normal conditions, so the standard approach is well-suited.

## 3. Estimating Absolute Rs vs. ΔRs (Incremental)

When adding $R_s$ into the state vector, you have a design choice: **estimate the absolute $R_s$** value directly, or estimate the **change in $R_s$** (Δ$R_s$) relative to a known baseline. Both approaches are viable, and in fact mathematically they are equivalent if set up properly (estimating Δ$R_s$ is essentially just a reparameterization of the absolute $R_s$ state). Here are some considerations:

* **Using Absolute $R_s$:** This means the EKF state $x_5$ (if we append as a 5th state) represents the actual resistance in ohms. You would initialize it to the known nominal value (for example, 0.037 Ω at room temp) with some uncertainty. The system model equations would include this state directly (e.g. $v_d - R_s i_d$ in the voltage equation). The advantage is simplicity – the state directly corresponds to the physical parameter. As the filter runs, $x_5$ will converge to the true $R_s$. This approach was used in references where EKF simultaneously identified resistance and flux; a good initial guess and proper noise tuning were important for convergence.

* **Using Δ$R_s$ (Change from Baseline):** This is analogous to what you have done with flux (estimating Δφ around a known map). In this scheme, you assume a baseline $R_{s0}$ (e.g. at 20°C) is known from design or initial calibration. The EKF state then represents the **deviation** from this baseline due to temperature rise. Your model equations would be slightly adjusted to use $R_{s} = R_{s0} + \Delta R_s$. For instance, the predicted $d$-axis current derivative becomes $\dot{i}_d = \frac{1}{L_d}\big(v_d - (R_{s0} + \Delta R_s)i_d + \omega(\phi_{q,\text{map}} + \Delta\phi_q)\big)$. The benefit here is that you are tracking a small value (Δ$R_s$ starts at 0 and remains relatively small, since $R_s$ won’t stray hugely from the nominal in normal operation). This can sometimes improve numerical stability or allow you to impose known bounds (e.g. Δ$R_s$ should remain positive and within a certain range). It also keeps consistency with the flux estimation approach – both flux and resistance are handled as **offsets** from known reference maps/values.

In practice, **either method can work**. If your initial nominal $R_s$ is very accurate, starting with Δ$R_s = 0$ (i.e. absolute state = baseline) is fine and the filter will adjust Δ$R_s$ as temperature changes. If the nominal might have some error, the EKF will correct that by ending up with a nonzero Δ$R_s$ (just as it would drive the absolute $R_s$ state toward the correct value). From an implementation standpoint, using absolute $R_s$ may be simpler (one less addition in the equations), but using Δ$R_s$ is conceptually neat when you have a known reference. The key is to initialize and tune the EKF properly in either case. Many implementations prefer directly estimating the parameter (absolute value), but since your flux is already in a Δ form relative to a LUT, it’s reasonable to also estimate $R_s$ as a deviation from a known $R_{s,\text{nominal}}$. This will not change the observability or performance in a significant way, as long as the filter knows the baseline. Just ensure the EKF’s state transition for $R_s$ (or Δ$R_s$) reflects that it’s essentially constant (with maybe a random-walk noise).

**Recommendation:** Given the similarity to your Δφ approach, you might lean towards a Δ$R_s$ state for consistency. But there is no strong disadvantage to estimating absolute $R_s$ directly. What matters more is the EKF tuning (initial covariance for $R_s$, process noise for $R_s$ state, etc.) and providing a reasonable initial value.

## 4. Feasibility at 500 µs Step Time (Computational & Implementation Outlook)

A 500 µs sampling period (2 kHz) is a very fast update rate for the EKF, which is excellent for tracking rapid electrical dynamics. Adding one extra state (going from 4 to 5 states) will have a minimal impact on computational load – the EKF equations (state prediction, Jacobian calculation, Kalman gain update) still involve small matrix operations (5x5 in this case). **No real-time issues are expected** if the 4-state filter was already running comfortably, especially since you mentioned ignoring processing constraints for now. Modern microcontrollers or DSPs used in motor drives can handle a 5-state EKF at 2 kHz loop in most cases (the operations scale roughly with $n^2$ for an EKF, so 5x5 vs 4x4 is only a small increase).

From an implementation perspective, you will need to update your state transition function and Jacobian to include $R_s$. In the continuous-time model, $R_s$ can be modeled as $\dot{R_s}=0$ (no change, except process noise). After discretization, that simply adds a state that stays the same across one step (plus noise). The Jacobian will have new partial derivatives: notably, the current derivative equations have $-\frac{\partial}{\partial R_s}(R_s i_d)/L_d = -i_d/L_d$ (and similarly $-i_q/L_q$ in the q-axis equation). This will populate the new column in the F matrix. These are straightforward to derive. In the measurement matrix $H$, $R_s$ does not appear (since we still measure only currents), so $H$ will have a 0 in the $R_s$ column. All other EKF steps remain similar.

One important consideration is **filter stability and convergence**: adding a new state can sometimes introduce convergence difficulties if the system’s measurements don’t sufficiently inform that state (i.e. observability issues). However, in this case, as discussed, there is a logical relationship: changes in $R_s$ produce measurable effects on both $i_d$ and $i_q$ (through voltage drops), whereas changes in flux produce different effects. As long as the motor is subjected to varying conditions (e.g. changes in torque or speed causing current variations), the filter should be able to observably separate $R_s$ and flux to a good extent. You can further **improve robustness** by occasionally injecting small test signals if needed (some methods inject a d-axis current pulse to identify $R_s$, but this may not be necessary in your application if normal operation provides enough excitation).

Lastly, running at 500 µs means the discrete-time model should use an appropriate discretization of the continuous dynamics. In your code, you use a first-order Euler ($F = I + A\,T_s$). This is acceptable given the very small step size (0.0005 s). If you include $R_s$, its discrete update would just be $R_{s,k+1} = R_{s,k}$ (plus noise), meaning $F_{R_s,R_s} = 1$. Keep the process noise for $R_s$ very low (perhaps similar magnitude to your flux process noise or based on expected variance per second of temperature change). With no heavy processing constraints, you have freedom to experiment with these tuning values to see the filter’s responsiveness.

**In summary**, the outlook of adding $R_s$ as a state in the EKF is positive. It is a well-grounded enhancement to capture temperature effects: it aligns with research that jointly estimates winding resistance and flux to maintain accuracy. The linear temperature model for $R_s$ is valid and can be used as prior knowledge. The EKF should reliably track slow changes in $R_s$; if changes are faster, it can adapt with proper noise tuning (at the cost of more measurement noise sensitivity). Whether you estimate absolute $R_s$ or Δ$R_s$, the filter equations only differ slightly – the choice can be based on convenience. And at a 500 µs loop, a 5-state EKF is very feasible. This approach should help ensure that **voltage errors due to $R_s$ variation are no longer mistakenly attributed solely to flux changes**, leading to more accurate flux (Δφ) estimation under temperature variations. The successful simulation and experiments in literature confirm that an EKF can accurately track both winding resistance and magnet flux simultaneously, which supports the viability of your proposed upgrade. Keep an eye on tuning and testing under various operating conditions, but overall this enhancement is a sound step forward.

**Sources:**

* Kerid *et al.*, *“Parameter identification of PMSM using EKF with temperature variation tracking...”*, demonstrating EKF tracking of stator winding resistance and magnet flux under temperature changes. This shows that accurate resistance estimation leads to precise flux estimation.
* Ibid., discussing how $R_s$ increases with copper temperature and magnet flux decreases with temperature, motivating online adaptation of these parameters.
* MathWorks Motor Control Blockset documentation (Runtime Parameter Estimation example), noting practical methods (EKF, RLS) for online estimation of flux linkage and $R_s$ in PMSMs, and suggesting injection-based tests for validating $R_s$ estimates.



## Rs  EKFへ追加　5状態ベクトル

以下では **Rs を絶対値のまま 5 番目の状態として追加** し、±30 % の物理上限も考慮した EKF に書き換えました。
（出力ポートは従来どおり 4 つのままにし、`Rs_est` は必要に応じて `x_est(5)` で取得できるようにしています。）

---

## 変更点まとめ

| 変更箇所                                | 主な修正内容                                                                        |
| ----------------------------------- | ----------------------------------------------------------------------------- |
| **状態ベクトル**                          | $[i_d,\; i_q,\; \Delta\phi_d,\; \Delta\phi_q,\; R_s]^\top$ （5 状態）             |
| **初期化**                             | `Rs_nom = 37e-3` Ω を 5 番目に設定。共分散 `P_est(5,5)` はやや大きめにして初期不確かさを表現。             |
| **プロセスノイズ $Q$**                     | 5 × 5 対角行列へ拡張。`Q_Rs` は ±30 % を数分程度で追従できる程度の値を目安に ≈ $10^{-10}\,\text{(Ω²/s)}$。 |
| **計測行列 $H$**                        | 5 列化（R\_s 列はゼロ）。                                                              |
| **予測関数 `f_discrete_delta_phiRs`**   | `Rs_state = x(5)` を使用して電流微分を計算。`Rs_next = Rs_state`（ランダムウォーク）。                |
| **ヤコビ行列 `calculate_F_delta_phiRs`** | 5 × 5 に拡張。<br>新しい偏導項:  $A_{1,5} = -i_d / L_d$, $A_{2,5} = -i_q / L_q$。        |
| **物理上限処理**                          | `Rs_pred` を `[0.7 Rs_nom, 1.3 Rs_nom]` にクリップして発散を防止。                          |

---

## 完全版コード

```matlab
%==================================================================
% ekf_delta_phi_estimator_Rs.m – STEP-1+ (Δφ + Rs) EKF
%==================================================================
function [delta_phi_d_est, delta_phi_q_est, P_diag, Id_est, Iq_est] = ...
    ekf_delta_phi_estimator_Rs(id_meas, iq_meas, vd, vq, omega, ...
                               phi_d_map, phi_q_map, ...
                               Ld_map, Lq_map, ...
                               Ldd, Ldq, Lqd, Lqq)

% ── constants ────────────────────────────────────────────────────
Rs_nom = 37e-3;                      % nominal stator resistance [Ω]
Rs_min = 0.7 * Rs_nom;               % −30 %
Rs_max = 1.3 * Rs_nom;               % +30 %
Ts     = 500e-6;                     % sample time [s]

% ── persistent state ────────────────────────────────────────────
persistent x_est P_est
if isempty(x_est)
    % [id iq Δφd Δφq Rs]
    x_est = [0; 0; 0; 0; Rs_nom];
    P_est = diag([0.01, 0.01, 1e-2, 1e-2,  (0.3*Rs_nom)^2]); % Rs ±30 %
end

% ── covariance matrices ─────────────────────────────────────────
Q = diag([1e-4, 1e-4, 1e-12, 1e-12, 1e-10]);   % process noise
R = diag([1e-3, 1e-3]);                        % sensor noise

% ── measurement matrix ──────────────────────────────────────────
H = [1 0 0 0 0;
     0 1 0 0 0];

% ── EKF – 1) prediction ─────────────────────────────────────────
u = [vd; vq; omega];

Ld_map = max(Ld_map, 1e-6);   % avoid divide-by-zero
Lq_map = max(Lq_map, 1e-6);

x_pred = f_discrete_delta_phiRs(x_est, u, Ts, ...
                                phi_d_map, phi_q_map, Ld_map, Lq_map, ...
                                Rs_min, Rs_max);

F = calculate_F_delta_phiRs(x_est, u, Ts, ...
                            Ld_map, Lq_map, Ldd, Ldq, Lqd, Lqq);

P_pred = F*P_est*F.' + Q;

% ── EKF – 2) update ─────────────────────────────────────────────
z = [id_meas; iq_meas];
y = z - H*x_pred;                       % innovation
S = H*P_pred*H.' + R;
K = (P_pred*H.') / S;                   % Kalman gain

x_est = x_pred + K*y;
P_est = (eye(5) - K*H) * P_pred;

% ── outputs ─────────────────────────────────────────────────────
delta_phi_d_est = x_est(3);
delta_phi_q_est = x_est(4);
Id_est          = x_est(1);
Iq_est          = x_est(2);
P_diag          = diag(P_est);

% (Rs_est  = x_est(5);  % ←必要なら取得)
end

%==================================================================
% 状態予測関数
%==================================================================
function x_next = f_discrete_delta_phiRs(x, u, Ts, ...
                                         phi_d_map, phi_q_map, ...
                                         Ld_map, Lq_map, ...
                                         Rs_min, Rs_max)
    id      = x(1);   iq   = x(2);
    dphi_d  = x(3);   dphi_q = x(4);
    Rs      = x(5);

    vd = u(1);  vq = u(2);  omega = u(3);

    % 電流微分方程式
    did_dt = (1/Ld_map) * (vd - Rs*id + omega*(phi_q_map + dphi_q));
    diq_dt = (1/Lq_map) * (vq - Rs*iq - omega*(phi_d_map + dphi_d));

    % 離散時間更新（Δφ, Rs はランダムウォーク）
    id_next     = id     + Ts * did_dt;
    iq_next     = iq     + Ts * diq_dt;
    dphi_d_next = dphi_d;      % random walk
    dphi_q_next = dphi_q;
    Rs_next     = Rs;          % random walk

    % 物理上限でクリップ（±30 %）
    Rs_next = min(max(Rs_next, Rs_min), Rs_max);

    x_next = [id_next; iq_next; dphi_d_next; dphi_q_next; Rs_next];
end

%==================================================================
% ヤコビ行列 F の計算関数
%==================================================================
function F = calculate_F_delta_phiRs(x, u, Ts, ...
                                     Ld_map, Lq_map, ...
                                     Ldd, Ldq, Lqd, Lqq)
    id = x(1);  iq = x(2);  Rs = x(5);
    omega = u(3);

    % 連続時間系ヤコビ A (5×5)
    A = zeros(5,5);

    % ∂(did_dt)/∂(...)
    A(1,1) = (-Rs + omega*Lqd) / Ld_map;
    A(1,2) = ( omega*Lqq )     / Ld_map;
    A(1,4) =  omega            / Ld_map;
    A(1,5) = -id               / Ld_map;   % new: d/dRs

    % ∂(diq_dt)/∂(...)
    A(2,1) = (-omega*Ldd)      / Lq_map;
    A(2,2) = (-Rs - omega*Ldq) / Lq_map;
    A(2,3) = -omega            / Lq_map;
    A(2,5) = -iq               / Lq_map;   % new: d/dRs

    % Δφ̇, Rṡ  are random walks → zeros elsewhere

    % 離散化 (Euler 1st-order): F = I + A*Ts
    F = eye(5) + A * Ts;
end
```

### 使い方メモ

* 既存モデルを呼び出している Simulink ブロックに置き換えるだけで動作します（出力数は同じ）。
* `Rs_est` をログしたい場合は `x_est(5)` を tap してください。
* ノイズ共分散 `Q(5,5)` を大きくすると Rs 追従が速くなりますが、観測雑音を追いかけた振動が増えるので注意してください。

これで **巻線抵抗の温度変化を EKF が同時に識別**でき、電圧降下誤差を Δφ の誤差に吸い込むリスクが大幅に低減します。
