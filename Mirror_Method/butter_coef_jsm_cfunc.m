Fs = 12e3; f_shift = 80; fc = 0.4*f_shift; order = 4;
[b,a]  = butter(order, fc/(Fs*0.5), 'low');
[~,gd] = grpdelay(b,a,1);          % gd [samples] at 0 rad/s
assignin('base','b_coeff', b);     % 1×5 row vector
assignin('base','a_coeff', a);
assignin('base','omega',   2*pi*f_shift);
assignin('base','gd_sec',  gd/Fs);
assignin('base','seqFlag', int32(+1));   % +1:正相
