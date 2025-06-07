classdef PhaseEstimator < handle
    properties
        Ts double
        filter   % DModuleFilter or DModuleFilter2
        Kp double
        Ki double
        theta_e double = 0.0
        omega_e double = 0.0
        err_int double = 0.0
    end
    methods
        function obj = PhaseEstimator(omega_h, Ts, Kp, Ki)
            if nargin < 3, Kp = 20; end
            if nargin < 4, Ki = 400; end
            obj.Ts     = Ts;
            obj.filter = DModuleFilter2(omega_h, Ts);   % ←厳密版
            obj.Kp     = Kp;
            obj.Ki     = Ki;
        end

        function [theta, s, c, omega, i_d, i_q] = update(obj, i_ab)
            [i_hi, i_hm] = obj.filter.update(i_ab);          %#ok<NASGU> (hm 未使用)

            % Park 変換（推定角）
            c = cos(obj.theta_e);   s = sin(obj.theta_e);
            i_d =  c*i_hi(1) + s*i_hi(2);
            % PLL 誤差（−i_d）
            err = -i_d;

            obj.err_int = obj.err_int + err*obj.Ts;
            obj.omega_e = obj.Kp*err + obj.Ki*obj.err_int;
            obj.theta_e = wrap_angle(obj.theta_e + obj.omega_e*obj.Ts);

            theta = obj.theta_e;
            omega = obj.omega_e;
        end
end
end
