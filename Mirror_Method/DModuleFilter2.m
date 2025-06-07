classdef DModuleFilter2 < handle
    properties
        Ts double
        omega_h double
        phi double = 0.0
        b_lp
        a_lp
        zi_i
        zi_m
    end
    methods
        function obj = DModuleFilter2(omega_h, Ts, lp_order, lp_cutoff_hz)
            if nargin < 3,  lp_order = 2;         end
            fs = 1/Ts;
            if nargin < 4
                lp_cutoff_hz = 0.25*omega_h/(2*pi);
            end
            Wc = lp_cutoff_hz/(fs/2);
            [obj.b_lp, obj.a_lp] = butter(lp_order, Wc, "low");

            M = max(length(obj.a_lp), length(obj.b_lp)) - 1;
            obj.zi_i = complex(zeros(M,1));
            obj.zi_m = complex(zeros(M,1));
            obj.Ts      = Ts;
            obj.omega_h = omega_h;
        end

        function [i_hi, i_hm] = update(obj, i_ab)
            % 実→複素
            i_c = i_ab(1) + 1j*i_ab(2);

            % 位相更新
            obj.phi = wrap_angle(obj.phi + obj.omega_h*obj.Ts);
            ejp = exp(-1j*obj.phi);
            ejm = exp(+1j*obj.phi);

            % 周波数シフト
            x_i = i_c.*ejp;
            x_m = i_c.*ejm;

            % LPF
            [y_i, obj.zi_i] = filter(obj.b_lp, obj.a_lp, x_i, obj.zi_i);
            [y_m, obj.zi_m] = filter(obj.b_lp, obj.a_lp, x_m, obj.zi_m);

            % 複素→実ベクトル
            i_hi = [real(y_i); imag(y_i)];
            i_hm = [real(y_m); imag(y_m)];
        end
    end
end
