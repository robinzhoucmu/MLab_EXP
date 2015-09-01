% Input: 
% push_vel: normalized 3*N pushing velocity.
% pt_contacts, pt_outward_normas: cell array of 2*2 contact info.
% mu: coefficient of friction at contact.
function [record] = PredictTwoPointsStable(push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs, lc_type)
num_pushes = size(push_vels, 2);
record.eps_norm = 0.01:0.001:0.25;

v_res = zeros(num_pushes, 1);
num_eps = length(record.eps_norm);
for i = 1:1:num_pushes
    [resnorm, x] = IsStable(push_vels(:,i), pt_contacts{i}, pt_outward_normals{i}, mu, lc_coeffs, pho, lc_type);
    v_res(i) = resnorm;
end
record.v_res = v_res;
for ind_eps = 1:1:num_eps
    eps_norm = record.eps_norm(ind_eps);    
    record.flag_stable_pred{ind_eps} = v_res < eps_norm;
end
end

