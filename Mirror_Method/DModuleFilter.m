classdef DModuleFilter < handle
    properties
        b_i
        a_i
        zi_alpha
        zi_beta
    end
    methods
        function obj = DModuleFilter(omega_h, Ts, zeta)
            if nargin < 3,  zeta = 0.1;  end
            fs = 1/Ts;
            f0 = omega_h/(2*pi);
            bw = f0*zeta;
            W  = [(f0-bw), (f0+bw)]/(fs/2);      % 正規化
            [obj.b_i, obj.a_i] = butter(2, W, "bandpass");

            M = max(length(obj.a_i), length(obj.b_i)) - 1;
            obj.zi_alpha = zeros(M,1);
            obj.zi_beta  = zeros(M,1);
        end

        function [i_hi, i_hm] = update(obj, i_ab)
            [yα, obj.zi_alpha] = filter(obj.b_i, obj.a_i, i_ab(1), obj.zi_alpha);
            [yβ, obj.zi_beta ] = filter(obj.b_i, obj.a_i, i_ab(2), obj.zi_beta );

            i_hi = [yα ; yβ];
            i_hm = -i_hi;   % 近似：鏡相 = −同相
        end
    end
end
