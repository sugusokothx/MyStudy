classdef BandStopFilter < handle
    properties
        b
        a
        zi  % 2×N 状態（α,β）
    end
    methods
        function obj = BandStopFilter(omega_h, Ts, Q)
            if nargin < 3, Q = 15; end
            fs = 1/Ts;
            f0 = omega_h/(2*pi);
            w0 = f0/(fs/2);
            BW = w0/Q;
            [obj.b, obj.a] = iirnotch(w0, BW);

            M = max(length(obj.a), length(obj.b)) - 1;
            obj.zi = zeros(2, M);
        end

        function dq_out = update(obj, dq_in)
            [d, obj.zi(1,:)] = filter(obj.b, obj.a, dq_in(1), obj.zi(1,:));
            [q, obj.zi(2,:)] = filter(obj.b, obj.a, dq_in(2), obj.zi(2,:));
            dq_out = [d ; q];
        end
    end
end
