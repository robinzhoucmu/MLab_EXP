clear all;
%close all;
%load results_lc; % black paper tested on poster paper.
load results_lc_2;  

index_lc = 3;
rng(50)  %

flag_output = 1;
pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 50;
mu = 0.95;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;
unit_scale = 1000;

num_gen_cors = 500;
num_verify = 20;
range_cor = 10;
rot_angle = 20; % in degree.
moveclose_dist = 0; % in mm. 
% One push scenario.
ContactInfo.approach_vectors = [-0.707107;
                                -0.707107;
                                0];
ContactInfo.push_points = [75;
                           75;
                           0];
% Three push scenario.                        
% ContactInfo.approach_vectors = [-0.707107, 0, 1;
%                                 -0.707107, 1, 0;
%                                 0,0, 0];
% ContactInfo.push_points = [75, 75, 0;
%                             75, 0, 75;
%                             0, 0, 0];

[push_actions] = GenerateRandomPushCORActionsGivenPushLocations(ContactInfo, num_gen_cors, pho * unit_scale, range_cor, rot_angle, moveclose_dist);
[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);
[push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale);

% eps_norm = 0:0.0005:0.01;
% eps_threshold = 0.0025;
% eps_threshold_index = int32(ceil((eps_threshold - eps_norm(1)) / (eps_norm(2) - eps_norm(1))));
eps_norm = eps;
eps_threshold_index = 1;

results = cell(4,1);
%--------------------------------------------------%
% cvx_poly4.
fprintf('Poly4CVX\n');
lc_coeffs_poly4_cvx = result_methods{index_lc}.coeffs{1};
[stable_pred_poly4_cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm);

flag_stable_threshold_poly4_cvx = stable_pred_poly4_cvx.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_polycvx = figure;
for ind_contact = 1:1:size(ContactInfo.push_points,2)
    finger_pts(:,2*ind_contact-1:2*ind_contact) = bsxfun(@plus, pt_contacts{1+(ind_contact-1) * num_gen_cors} * unit_scale, trans(1:2));
end
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_poly4_cvx, h_polycvx);
indices_out = [];
if (flag_output)
    file_name_poly4_stable = 'push_actions_range4_random.txt';
    [indices_output] = WritePushActionToDisk(push_actions, file_name_poly4_stable, ones(num_gen_cors * 3, 1), num_verify);
    %file_name_poly4_nonstable = 'push_actions_range4_poly4_nonstable.txt';  
    %[indices_poly4_nonstable_output] = WritePushActionToDisk(push_actions, file_name_poly4_nonstable, ones(num_gen_cors, 1), num_verify);
end
result_compare(:,1) = flag_stable_threshold_poly4_cvx(indices_output)

%--------------------------------------------------%

fprintf('Poly4\n');
lc_coeffs_poly4 = result_methods{index_lc}.coeffs{2};
[stable_pred_poly4] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4, 'poly4', eps_norm);

flag_stable_threshold_poly4 = stable_pred_poly4.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_poly4 = figure;
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_poly4, h_poly4);
 
% Check what the poly4 model will agree with the poly4_cvx model.
result_compare(:,2) = flag_stable_threshold_poly4(indices_output)
%flag_stable_threshold_poly4(indices_poly4_nonstable_output)




fprintf('quadratic\n');
lc_coeffs_quad = result_methods{index_lc}.coeffs{3};
[stable_pred_quad] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_quad, 'quadratic', eps_norm);

flag_stable_threshold_quad = stable_pred_quad.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_quad = figure;
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_quad, h_quad);

%--------------------------------------------------%
% Check what the qp model will agree with the poly4_cvx model.
result_compare(:,3) = flag_stable_threshold_quad(indices_output)
%flag_stable_threshold_quad(indices_poly4_nonstable_output)

fprintf('gaussianprocess\n');
lc_coeffs_gp = result_methods{index_lc}.coeffs{4};
F_train = result_methods{index_lc}.F_train;
V_train = result_methods{index_lc}.V_train;
gp_record.F_train = bsxfun(@rdivide, F_train, sqrt(sum(F_train.^2)));
gp_record.V_train = V_train;
gp_record.coeffs = result_methods{index_lc}.coeffs{4};
[stable_pred_gp] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, gp_record, 'gp', eps_norm);

flag_stable_threshold_gp = stable_pred_gp.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_gp = figure;
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_gp, h_gp)
result_compare(:,4) = flag_stable_threshold_gp(indices_output)
%flag_stable_threshold_gp(indices_poly4_nonstable_output)
result_compare = int32(result_compare);

length(find(flag_stable_threshold_poly4_cvx == 1)) / length(flag_stable_threshold_poly4_cvx)
length(find(flag_stable_threshold_poly4_cvx(indices_output) == 1)) / length(flag_stable_threshold_poly4_cvx(indices_output))
% flag_diff = xor(flag_stable_threshold_poly4_cvx, flag_stable_threshold_quad);
% ind_diff = find(flag_diff == 1);
% length(ind_diff)
% pred_diff_poly4 = flag_stable_threshold_poly4_cvx(ind_diff);
% pred_diff_quad = flag_stable_threshold_quad(ind_diff);
% file_name = 'push_action_diff.txt';
% WritePushActionToDisk(push_actions, file_name, flag_diff);
