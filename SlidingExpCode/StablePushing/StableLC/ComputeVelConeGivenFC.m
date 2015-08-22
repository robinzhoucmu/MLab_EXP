% Input: 
% fc_edges: 3*(2N) edges of the friction cone.
% lc_coeffs: limit surface coefficients.
% lc_type: 'quadratic' or 'poly4'.
% Output:
% vc_edges: 3*(2N) edges of the velocity cone.
function [ vc_edges ] = ComputeVelConeGivenFC(fc_edges, lc_coeffs, lc_type)
if strcmp(lc_type, 'quadratic')
    A = lc_coeffs;
    vel = A * fc_edges;
    dir_vel = bsxfun(@rdivide, vel, sqrt(sum(vel.^2, 1)));
    vc_edges = dir_vel;
elseif strcmp(lc_type, 'poly4')
    [dir_vel, vel] = GetFrom4thOrderPoly(lc_coeffs, fc_edges);
    vc_edges = dir_vel';
end

end

