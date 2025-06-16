function [y_alpha, y_beta] = VRAF(u_alpha, u_beta, f_shift, Fs, seq, fc)
%#codegen

% VRAF: Vector-Rotator-Assisted Filter for alpha-beta two-phase signals.
%
% Inputs:
%   u_alpha, u_beta: input signals (scalar samples per step)
%   f_shift: target harmonic frequency [Hz]
%   Fs: sampling frequency [Hz]
%   seq: +1 for positive-sequence, -1 for negative-sequence
%   fc: cutoff frequency for IIR LPF [Hz]
%
% Outputs:
%   y_alpha, y_beta: filtered signals

persistent b a zR zI t_idx omega initialized

if isempty(initialized)
    order = 4; % Butterworth filter order (even)
    omega = 2*pi*f_shift;

    Wn = fc/(Fs/2);
    [b,a] = butter(order, Wn, 'low');

    % Initialize filter states (real and imaginary separately)
    max_len = max(length(a), length(b)) - 1;
    zR = zeros(max_len, 1);
    zI = zeros(max_len, 1);
    
    t_idx = 0;
    initialized = true;
end

% Current time calculation
t = t_idx / Fs;
t_idx = t_idx + 1;

% Complex input
u_p = u_alpha + 1i * u_beta;

% Demodulation (rotation)
if seq == 1   % positive-sequence extraction
    u_tilde = u_p * exp(-1i*omega*t);
    rot_back = exp(1i*omega*t);
else          % negative-sequence extraction
    u_tilde = u_p * exp(1i*omega*t);
    rot_back = exp(-1i*omega*t);
end

% Separate into real and imaginary
uR = real(u_tilde);
uI = imag(u_tilde);

% Apply IIR low-pass filter separately (real part)
[yR, zR] = filter(b, a, uR, zR);

% Apply IIR low-pass filter separately (imag part)
[yI, zI] = filter(b, a, uI, zI);

% Remodulate back
y_p = (yR + 1i*yI) * rot_back;

% Output real and imag parts separately
y_alpha = real(y_p);
y_beta  = imag(y_p);
