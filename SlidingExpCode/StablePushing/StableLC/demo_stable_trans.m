clear all; close all;
log_file_stable_sensor = 'Logs/30_90_30_30_30_90/exp_Fri Aug 28 14:13:40 2015_sensor20.txt';
log_file_stable_pushactions = 'Logs/30_90_30_30_30_90/exp_Fri Aug 28 14:13:40 2015_pushaction20.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';

pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 0.05;
mu = 1;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
unit_scale = 1000;

% Extract from poking log. 
[record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

ratio_val = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

num_train_all = size(slider_velocities_train, 1);
ratio_train = 1;
num_train = floor(num_train_all * ratio_train);
fprintf('*********\nUse training size:%d\n', num_train);
train_data.V = slider_velocities_train(1:num_train, :);
train_data.F = push_wrenches_train(1:num_train, :);
            
[result_methods] = MethodComparision(train_data, validation_data);

% Extract from two points pushing log. 
R_tool_two_points = -R_tool_point;
[record_log_two_points] = ExtractFromLog(log_file_stable_sensor, pho, R_tool_two_points, H_tf, unit_scale);

[push_actions] = ParsePushActionLog(log_file_stable_pushactions);
% Decide translational stability. Note that push_vectors do not need to be
% normalized because angular part will be 0.
dev_angles = acos(diag(record_log_two_points.slider_velocities * push_actions.push_vectors)) * 180 / pi;
flag_stable_exp = dev_angles < 5;

R_z = [cos(pi/2), -sin(pi/2), 0;
       sin(pi/2), cos(pi/2), 0;
       0, 0, 1];

lc_coeffs = result_methods.coeffs{1};
lc_type = 'poly4';

lc_coeffs = result_methods.coeffs{3};
lc_type = 'quadratic';
% Check for poly4 convex prediction.
num_pushes = size(push_actions.push_vectors, 2);
v_res = zeros(num_pushes, 1);
for i = 1:1:num_pushes
    vel = push_actions.push_vectors(:,i);
    d = R_z * push_actions.approach_vectors(:,i) * fingers_width / 2;
    fingers_pos = [push_actions.push_points(:,i) - d, push_actions.push_points(:,i) + d];
    fingers_pos = bsxfun(@minus, fingers_pos, trans);
    pt_contacts = fingers_pos(1:2,:) / unit_scale;
    pt_outward_normals = repmat(-push_actions.approach_vectors(1:2, i), [1,2]);
    [resnorm, x] = IsStable(vel, pt_contacts, pt_outward_normals, mu, lc_coeffs, pho, lc_type);
    v_res(i) = resnorm;
end
flag_stable_predict = v_res < 0.1;