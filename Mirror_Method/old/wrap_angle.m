function thetaWrapped = wrap_angle(theta)
% 正規化：[-pi, pi]
thetaWrapped = mod(theta + pi, 2*pi) - pi;
end
