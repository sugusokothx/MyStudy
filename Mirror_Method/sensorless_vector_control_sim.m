% Sensorless Vector Control – MATLAB Simulation
clear; clc; close all;

%% パラメータ
Ts       = 1/12000;          % 10 kHz
omega_h  = 2*pi*1000;        % 1 kHz HF
Vh       = 20;               % HF 電圧振幅
polePairs = 4;

R  = 0.03;
Ld = 1e-3;
Lq = 1.5e-3;
theta_true = 30/180*pi;      % [rad] 固定

steps = floor(1.0/Ts);

%% モジュール生成
hfv       = HFVGenerator(Vh, omega_h, Ts);
phaseEst  = PhaseEstimator(omega_h, Ts);
speedEst  = SpeedEstimator(polePairs);
notch     = BandStopFilter(omega_h, Ts);

%% 事前確保
theta_true_log = zeros(steps,1);
theta_est_log  = zeros(steps,1);
time_log       = Ts*(0:steps-1)';

i_alpha_log = zeros(steps,1);
i_beta_log  = zeros(steps,1);
v_alpha_log = zeros(steps,1);
v_beta_log  = zeros(steps,1);
i_mag_log   = zeros(steps,1);
phi_h_log   = zeros(steps,1);
id_log      = zeros(steps,1);
iq_log      = zeros(steps,1);

%% ループ
id_prev = 0; iq_prev = 0;

for k = 1:steps
    % HFV 生成
    v_hf = hfv.update();
    v_alpha = v_hf(1); v_beta = v_hf(2);

    % 真の座標系→dq
    cos_t = cos(theta_true); sin_t = sin(theta_true);
    Vd =  cos_t*v_alpha + sin_t*v_beta;
    Vq = -sin_t*v_alpha + cos_t*v_beta;

    % dq 電流オイラー積分
    id_new = id_prev + (Ts/Ld)*(Vd - R*id_prev);
    iq_new = iq_prev + (Ts/Lq)*(Vq - R*iq_prev);

    % dq→αβ
    i_alpha =  cos_t*id_new - sin_t*iq_new;
    i_beta  =  sin_t*id_new + cos_t*iq_new;
    i_ab    = [i_alpha; i_beta];

    % 位相推定
    [theta_est,~,~,omega_e,i_d_est,i_q_est] = phaseEst.update(i_ab);

    %% ログ
    theta_true_log(k) = theta_true;
    theta_est_log(k)  = theta_est;
    i_alpha_log(k)    = i_alpha;
    i_beta_log(k)     = i_beta;
    v_alpha_log(k)    = v_alpha;
    v_beta_log(k)     = v_beta;
    i_mag_log(k)      = hypot(i_alpha, i_beta);
    phi_h_log(k)      = hfv.phi_h;
    id_log(k)         = i_d_est;
    iq_log(k)         = i_q_est;

    %% フィルタ / 速度（必要なら）
    omega_m = speedEst.update(omega_e); %#ok<NASGU>
    dq_filt = notch.update([id_new; iq_new]);          %#ok<NASGU>

    id_prev = id_new;
    iq_prev = iq_new;
end

%% 可視化
figure;
plot(time_log, rad2deg(theta_true_log), 'LineWidth',1.5); hold on;
plot(time_log, rad2deg(theta_est_log ), '--', 'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Angle [deg]');
title('Estimated vs. True Angle');
grid on; legend('True','Estimated');
