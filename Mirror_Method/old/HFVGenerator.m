classdef HFVGenerator < handle
    properties
        Vh double
        omega_h double
        Ts double
        phi_h double = 0.0
    end
    methods
        function obj = HFVGenerator(Vh, omega_h, Ts)
            obj.Vh      = Vh;
            obj.omega_h = omega_h;
            obj.Ts      = Ts;
        end
        function v = update(obj)
            obj.phi_h = wrap_angle(obj.phi_h - obj.omega_h*obj.Ts);   % −符号＝逆回転
            v_alpha   = obj.Vh*cos(obj.phi_h);
            v_beta    = obj.Vh*sin(obj.phi_h);
            v         = [v_alpha ; v_beta];
        end
    end
end
