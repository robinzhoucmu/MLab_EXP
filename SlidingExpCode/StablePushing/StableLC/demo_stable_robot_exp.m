clear all; 
%close all;
rng(1);

log_file_stable_sensor = '30_90_30_30_30_90/exp_Mon Aug 31 17:26:31 2015_sensor50.txt';
log_file_stable_pushactions = '30_90_30_30_30_90/exp_Mon Aug 31 17:26:31 2015_pushaction50.txt';


%log_file_stable_sensor = '30_63.33_43.33_30_43.33_63.33/exp_Mon Aug 31 14:33:34 2015_sensor50.txt';
%log_file_stable_pushactions = '30_63.33_43.33_30_43.33_63.33/exp_Mon Aug 31 14:33:34 2015_pushaction50.txt';

%log_file_stable_sensor = 'patch/exp_Mon Aug 31 16:06:23 2015_sensor50.txt';
%log_file_stable_pushactions = 'patch/exp_Mon Aug 31 16:06:23 2015_pushaction50.txt';

%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/patch/exp_08_18_1435_50.txt';


eps_norm_poly4 = 0.05;
eps_norm_quad = 0.05;
eps_norm_gp = 0.05;

pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 0.05;
mu = 1.0;
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
%log_file_stable_sensor = '30_90_30_30_30_90/exp_Mon Aug 31 22:04:48 2015_sensor15.txt';
%log_file_stable_pushactions = '30_90_30_30_30_90/exp_Mon Aug 31 22:04:48 2015_pushaction15.txt';
%flag_stable_empirical([4,5,6,8,14]) = 1;

%log_file_stable_sensor = 'patch/exp_Mon Aug 31 23:08:46 2015_sensor20.txt';
%log_file_stable_pushactions = 'patch/exp_Mon Aug 31 23:08:46 2015_pushaction20.txt';
%flag_stable_empirical([3,5,12,16,17,19]) = 1;


log_file_stable_sensor = '30_63.33_43.33_30_43.33_63.33/exp_Mon Aug 31 23:52:30 2015_sensor20.txt';
log_file_stable_pushactions = '30_63.33_43.33_30_43.33_63.33/exp_Mon Aug 31 23:52:30 2015_pushaction20.txt';


eps_stable_angle = 5 * pi/180;
eps_stable_trans = 10 / 1000;

[record_log_two_points] = ExtractFromLog(log_file_stable_sensor, pho, R_tool_two_points, H_tf, unit_scale);
[push_actions] = ParsePushActionLog(log_file_stable_pushactions);

[push_vels,  flag_stable_empirical_dummy, dev_angles, disp_cor, disp_diff] = ExtractVelAndEmpiricalStablity(...
    push_actions, record_log_two_points, H_tf, pho,  unit_scale, eps_stable_angle, eps_stable_trans);

flag_stable_empirical = zeros(size(push_vels,2), 1);
flag_stable_empirical([3, 5,7,10,12,13,15,16,19]) = 1;


fprintf('Ratio of stable pushes:%f\n', length(find(flag_stable_empirical == 1)) / length(flag_stable_empirical));



[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);

h = figure;
results = cell(4,1);
% cvx_poly4.
lc_coeffs_poly4_cvx = result_methods.coeffs{1};
[stable_pred_poly4_cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4');

[result_poly4_cvx] = ComputePRCurve(stable_pred_poly4_cvx, flag_stable_empirical);
results{1} = result_poly4_cvx;
%poly4.
 lc_coeffs_poly4 = result_methods.coeffs{2};
 [stable_pred_poly4] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4, 'poly4');

[result_poly4] = ComputePRCurve(stable_pred_poly4, flag_stable_empirical);
results{2} = result_poly4;
 
%quad.
lc_coeffs_quad = result_methods.coeffs{3};
[stable_pred_quad] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_quad, 'quadratic');

[result_quad] = ComputePRCurve(stable_pred_quad, flag_stable_empirical);
results{3} = result_quad;

%gp.
gp_record.F_train = bsxfun(@rdivide, train_data.F, sqrt(sum(train_data.F.^2)));
gp_record.V_train = train_data.V;
gp_record.coeffs = result_methods.coeffs{4};
[stable_pred_gp] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, gp_record, 'gp');

[result_gp] = ComputePRCurve(stable_pred_gp, flag_stable_empirical);
results{4} = result_gp;

DrawPRCurves(results, h);
idx_stable_empirical = find(flag_stable_empirical == 1)
%idx_stable_poly4cvx = find(flag_stable_pred_poly4cvx == 1)
%idx_stable_poly4 = find(flag_stable_pred_poly4 == 1)
% idx_stable_quad = find(flag_stable_pred_quad == 1)
% idx_stable_gp = find(flag_stable_pred_gp == 1)
% 
stable_pred_poly4_cvx.v_res(idx_stable_empirical)
stable_pred_poly4.v_res(idx_stable_empirical)
stable_pred_quad.v_res(idx_stable_empirical)
stable_pred_gp.v_res(idx_stable_empirical)
stable_vels = push_vels(:, idx_stable_empirical)
% v_res_poly4(idx_stable_empirical)
% v_res_quad(idx_stable_empirical)
% v_res_gp(idx_stable_empirical)
