clear all;
rng(1);

pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
unit_scale = 1000;   
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
Tri_com = [0.15/3; 0.15/3];
% if ration_ration == 1 then we will have 40 training data.            
ratio_train = 0.5;

%---------------------------------------------------------------------%
%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
% Extract from poking log. 
[record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

ratio_val = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

num_train_all = size(slider_velocities_train, 1);
num_train = floor(num_train_all * ratio_train);
index_train = randperm(num_train_all);
fprintf('*********\nUse training size:%d\n', num_train);
train_data.V = slider_velocities_train(index_train(1:num_train), :);
train_data.F = push_wrenches_train(index_train(1:num_train), :);
            
[result_methods{1}] = MethodComparision(train_data, validation_data);
Tri_pts{1} = [0.03 0.09 0.03; 0.03 0.03 0.09];

%---------------------------------------------------------------------%
%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/10_90_10_10_30_130/exp_08_15_50.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/wood_10_90_10_10_30_130/exp_08_18_1100_50.txt';
% Extract from poking log. 
[record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

ratio_val = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

num_train_all = size(slider_velocities_train, 1);
num_train = floor(num_train_all * ratio_train);
index_train = randperm(num_train_all);
fprintf('*********\nUse training size:%d\n', num_train);
train_data.V = slider_velocities_train(index_train(1:num_train), :);
train_data.F = push_wrenches_train(index_train(1:num_train), :);
            
[result_methods{2}] = MethodComparision(train_data, validation_data);

%-----------------------------------------------------------------------%
%log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/patch/exp_08_18_1435_50.txt';
log_file_poke = '../../FusionMatlab/RandPushExp/SensorLogs/wood_patch/exp_08_17_0839_50.txt';

% Extract from poking log. 
[record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

ratio_val = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

num_train_all = size(slider_velocities_train, 1);
num_train = floor(num_train_all * ratio_train);
index_train = randperm(num_train_all);
fprintf('*********\nUse training size:%d\n', num_train);
train_data.V = slider_velocities_train(index_train(1:num_train), :);
train_data.F = push_wrenches_train(index_train(1:num_train), :);
            
[result_methods{3}] = MethodComparision(train_data, validation_data);

save('results_lc_2', 'result_methods');






