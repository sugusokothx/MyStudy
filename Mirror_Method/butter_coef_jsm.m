% Butterworth LPF coefficient calculation for VRAF
% Author: [Your Name]
% Date: [Today's Date]

%% Parameters Setting
Fs = 12000;          % Sampling frequency [Hz]
fc = 150;            % Cut-off frequency [Hz]
order = 4;           % Butterworth filter order (even)

%% Normalized cutoff frequency
Wn = fc / (Fs/2);    % Normalized cutoff frequency (Nyquist = Fs/2)

%% Butterworth LPF design
[b, a] = butter(order, Wn, 'low');

%% Display coefficients (for C implementation)
fprintf('Butterworth filter coefficients (C-ready):\n\n');

fprintf('float b[%d] = {', length(b));
fprintf('%ff, ', b(1:end-1));
fprintf('%ff};\n', b(end));

fprintf('float a[%d] = {', length(a));
fprintf('%ff, ', a(1:end-1));
fprintf('%ff};\n', a(end));
