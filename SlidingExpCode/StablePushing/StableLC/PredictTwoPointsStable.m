% Input: 
% push_vel: normalized 3*N pushing velocity.
% pt_contacts, pt_outward_normas: cell array of 2*2 contact info.
% mu: coefficient of friction at contact.
function [ flag_stable_pred, v_res] = PredictTwoPointsStable(push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs, lc_type, eps_norm)
if (nargin < 8)
 eps_norm = 0.1;
end
num_pushes = size(push_vels, 2);
v_res = zeros(num_pushes, 1);
for i = 1:1:num_pushes
    [resnorm, x] = IsStable(push_vels(:,i), pt_contacts{i}, pt_outward_normals{i}, mu, lc_coeffs, pho, lc_type);
    v_res(i) = resnorm;
end
flag_stable_pred = v_res < eps_norm;
end

