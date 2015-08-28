% Input: 
% theta, phi of spherical coordinate for force direction.
% V: 3*1, input velocity.
function [err] = EvaluateForceDirectionPoly4(angle, coeffs, V)
V = V / norm(V);
% Spherical coordinate.
theta = angle(1);
phi = angle(2);
f = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
Vp = GetVelFrom4thOrderPoly(coeffs, f)';
err = (Vp - V)' * (Vp-V);
end

