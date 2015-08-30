clear all; close all;
rng(1);
log_file_stable_sensor = '30_90_30_30_30_90/exp_Sat Aug 29 21:24:36 2015_sensor25.txt';
log_file_stable_pushactions = '30_90_30_30_30_90/exp_Sat Aug 29 21:24:36 2015_pushaction25.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';

eps_stable_angle = 10;
eps_norm_poly4 = 0.05;
eps_norm_quad = 0.05;

pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 0.05;
mu = 1;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;
unit_scale = 1000;

% Extract from poking log. 
[record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

ratio_val = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

num_train_all = size(slider_velocities_train, 1);
ratio_train = 1.0;
num_train = floor(num_train_all * ratio_train);
fprintf('*********\nUse training size:%d\n', num_train);
train_data.V = slider_velocities_train(1:num_train, :);
train_data.F = push_wrenches_train(1:num_train, :);
            
[result_methods] = MethodComparision(train_data, validation_data);

%-------------------------------------------------------------------------------------%
%-------------------------------------------------------------------------------------%
[record_log_two_points] = ExtractFromLog(log_file_stable_sensor, pho, R_tool_two_points, H_tf, unit_scale);
[push_actions] = ParsePushActionLog(log_file_stable_pushactions);

[push_vels,  flag_stable_empirical] = ExtractVelAndEmpiricalStablity(...
    push_actions, record_log_two_points, H_tf, pho,  unit_scale, eps_stable_angle);

[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);

% cvx_poly4.
lc_coeffs_poly4_cvx = result_methods.coeffs{1};
 [ flag_stable_pred_poly4cvx, v_res_poly4cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm_poly4);
%poly4.
 lc_coeffs_poly4 = result_methods.coeffs{2};
 [ flag_stable_pred_poly4, v_res_poly4] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4, 'poly4', eps_norm_poly4);
%quad.
lc_coeffs_quad = result_methods.coeffs{3};
[ flag_stable_pred_quad, v_res_quad] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_quad, 'quadratic', eps_norm_quad);
 
idx_stable_empirical = find(flag_stable_empirical == 1)
idx_stable_poly4cvx = find(flag_stable_pred_poly4cvx == 1)
idx_stable_poly4 = find(flag_stable_pred_poly4 == 1)
idx_stable_quad = find(flag_stable_pred_quad == 1)

v_res_poly4cvx(idx_stable_empirical)
v_res_poly4(idx_stable_empirical)
v_res_quad(idx_stable_empirical)
