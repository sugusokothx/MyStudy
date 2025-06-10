classdef SpeedEstimator < handle
    properties
        Np double
    end
    methods
        function obj = SpeedEstimator(polePairs)
            obj.Np = polePairs;
        end
        function omega_m = update(obj, omega_e)
            omega_m = omega_e/obj.Np;
        end
    end
end
