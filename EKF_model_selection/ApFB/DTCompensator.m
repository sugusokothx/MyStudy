function [vdt_a, vdt_b, vdt_c, alpha_out, C_out] = DTCompensator( ...
        ia, ib, ic, vdc, theta_e, ENABLE_ADAPT)
%#codegen
%------------------------------------------------------------------
% Online dead-time compensator with two-parameter model
%  (c) 2025  for prototype evaluation
%
% Inputs
%   ia, ib, ic   [A]   : phase currents
%   vdc          [V]   : DC-link voltage
%   theta_e      [rad] : electrical angle (for dq transform)
%   ENABLE_ADAPT (bool): true -> update α, C
%
% Outputs
%   vdt_a,b,c [V] : compensation voltages to add to inverter commands
%   alpha_out,C_out : current estimates of α, C
%------------------------------------------------------------------

%% === user-editable constants ====================================
Td       = 2e-6;        % Dead-time (s)  ― デバイス仕様を入力
f_car    = 10e3;        % PWM carrier frequency (Hz)
Ts       = 1/f_car;     % control period (s)

kAlpha   = 2e-4;        % adaptation gain for α  (small → slow)
kC       = 2e-10;       % adaptation gain for C  (small → slow)

alpha_min = 0.8;        % practical limits
alpha_max = 1.2;
C_min     = 0;
C_max     = 4e-6;

lpf_fc   = 200;         % LPF cut-off for fundamental (Hz)
%% ================================================================

persistent alpha C id_lp iq_lp
if isempty(alpha)
    alpha  = 1.00;      % initial guess
    C      = 1e-6;      % initial guess (≈寄生容量 / デバイス)
    id_lp  = 0;
    iq_lp  = 0;
end

%% --- dead-time compensation for each phase ----------------------
i_vec   = [ia, ib, ic];
sgn_i   = sign(i_vec);

% current-dependent term threshold i_x
ix  = C*vdc/(alpha*Td);         % eq.(7)
den = abs(i_vec) + ix;          % avoid div-by-0

% voltage offset (eq.(8))  — 符号で分岐
vdt = -alpha*Td*f_car*vdc.*sgn_i ...
      + (C*f_car*vdc.^2).*(sgn_i./den);

vdt_a = vdt(1);
vdt_b = vdt(2);
vdt_c = vdt(3);

%% --- α, C online tuning  ----------------------------------------
% abc -> dq (motor convention, 2/3 factor for amplitude invariance)
cos_t = cos(theta_e);          sin_t = sin(theta_e);
cos_t120 = cos(theta_e - 2*pi/3);  sin_t120 = sin(theta_e - 2*pi/3);
cos_t240 = cos(theta_e + 2*pi/3);  sin_t240 = sin(theta_e + 2*pi/3);

id  = (2/3)*( cos_t   *ia + cos_t120 *ib + cos_t240 *ic );
iq  = (2/3)*( -sin_t  *ia - sin_t120 *ib - sin_t240 *ic );

% first-order LPF for fundamental (simple IIR)
omega_c = 2*pi*lpf_fc;
alpha_lpf = Ts*omega_c / (1 + Ts*omega_c);
id_lp = id_lp + alpha_lpf*(id - id_lp);
iq_lp = iq_lp + alpha_lpf*(iq - iq_lp);

% ripple components (≈ high-freq inc. 6th harm.)
idr = id - id_lp;
iqr = iq - iq_lp;

% Error signal = ripple magnitude along q-axis in synchronous frame
err =  idr.*(-sin_t) + iqr.*cos_t;   % heuristic → reduce torque ripple

if ENABLE_ADAPT
    % α adapts over entire current range
    alpha = alpha - kAlpha * err * Ts;
    
    % C adapts only near zero current (|i|<4*ix) to decouple parameters
    if abs(ia)+abs(ib)+abs(ic) < 4*abs(ix)
        C = C - kC * err * Ts;
    end
end

% saturation
alpha = min(max(alpha, alpha_min), alpha_max);
C     = min(max(C,     C_min),     C_max);

alpha_out = alpha;
C_out     = C;

%% ====================  END  =====================================
%  Integration note:
%   1. Add vdt_* to phase-voltage references **before** SVPWM.
%   2. Tune kAlpha, kC with ENABLE_ADAPT=false first (open-loop check).
%   3. Gradually raise gains until 5th/7th current harmonics on FFT drop.
%------------------------------------------------------------------
end
