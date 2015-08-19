clear all; close all;
%rng(1);

%log_file_name = 'SensorLogs/10_90_10_10_30_130/exp_08_15_50.txt';
%log_file_name = 'SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';
%log_file_name = 'SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';

%log_file_name = 'SensorLogs/wood_10_130_10_10_10_130/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/wood_patch/exp_08_17_0839_50.txt';
%log_file_name = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
log_file_name = 'SensorLogs/wood_10_90_10_10_30_130/exp_08_18_1100_50.txt';

Tri_V = [0,0.15,0;0,0,0.15];
unit_scale = 1000;
H_tf = eye(4,4);
trans = [50;50;0];
H_tf = [eye(3,3), trans;
      0,0,0,1];
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';
      
% Parameters for trianglular block.         
Tri_mass = 1.518;
mu_f = 4.2 / (Tri_mass * 9.8);
Tri_pressure = Tri_mass * mu_f;
Tri_com = [0.15/3; 0.15/3];

% support Points

%Tri_pts = [0.03    0.09    0.03; 0.03    0.03    0.09];
Tri_pts = [0.01, 0.09,0.01;0.01,0.03,0.13];
%Tri_pts = [0.03, 0.06333, 0.04333; 0.03, 0.04333, 0.06333];
%Tri_pts = [0.01, 0.01, 0.13; 0.01,0.13,0.01];
%[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, Tri_com, Tri_mass);

Tri_pts_cp = Tri_pts;

Tri_pts = bsxfun(@minus, Tri_pts, Tri_com);
[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, [0;0], Tri_pressure);
Tri_pho = 0.05;
[ record_log ] = ExtractFromLog( log_file_name, Tri_pho, R_tool, H_tf, unit_scale);

ratio_validation = 0.2;
[slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
    SplitTrainTestData(record_log.slider_velocities, record_log.push_wrenches, 1 - ratio_validation);
validation_data.V = slider_velocities_val;
validation_data.F = push_wrenches_val;

% Sample from the ideal pressure distribution as test data. 
Nc = 402;
CORs = GenerateRandomCORs3(Tri_pts, Nc, 402/3);
[F, bv] = GenFVPairsFromPD(Tri_pts, Tri_pds, CORs);
% Change to row representation.
F = F';
pho=Tri_pho;
[bv, F] = NormalizeForceAndVelocities(bv, F, pho);

r = [0.25, 0.5, 0.75, 1.0];
num_train_all = size(slider_velocities_train, 1);
for ind_ratio = 1:1:length(r)
    train_data.V = slider_velocities_train(1:floor(num_train_all * r(ind_ratio)), :);
    train_data.F = push_wrenches_train()
end



