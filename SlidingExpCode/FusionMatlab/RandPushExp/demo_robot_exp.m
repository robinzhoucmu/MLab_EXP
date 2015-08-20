clear all; close all;
%rng(1);
tic;

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

h_tri = DrawTriangle(Tri_V, Tri_com, Tri_pts_cp, Tri_pds);


[ record_log ] = ExtractFromLog( log_file_name, Tri_pho, R_tool, H_tf, unit_scale);

% Sample from the ideal pressure distribution as test data. 
Nc = 603;
CORs = GenerateRandomCORs3(Tri_pts, Nc, 402/3);
[F, bv] = GenFVPairsFromPD(Tri_pts, Tri_pds, CORs);
% Change to row representation.
F = F';
pho=Tri_pho;
[bv, F] = NormalizeForceAndVelocities(bv, F, pho);
testsim_data.V = bv;
testsim_data.F = F;

num_evals = 10;
for ind_eval = 1:1:num_evals
    fprintf('-----------------------\n');
    fprintf('percentage completed:%f\n', (ind_eval-1) * 100 / num_evals);
    fprintf('-----------------------\n');
    % Split out test data. 50*0.2 = 10;
    ratio_test = 0.2;
    [slider_velocities_train_val, slider_velocities_test, push_wrenches_train_val, push_wrenches_test] = ...
        SplitTrainTestData(record_log.slider_velocities, record_log.push_wrenches, 1 - ratio_test);
    testexp_data.V = slider_velocities_test;
    testexp_data.F = push_wrenches_test;

    % Split out validation data from experiment data. 
    % (1 - ratio_test) * num_data * ratio_val
    % 40*0.25 = 10;
    ratio_val = 0.25;
    [slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
         SplitTrainTestData(slider_velocities_train_val, push_wrenches_train_val, 1 - ratio_val);
    validation_data.V = slider_velocities_val;
    validation_data.F = push_wrenches_val;

    r = [1/3, 2/3, 1.0];
    num_methods = 4;
    num_train_all = size(slider_velocities_train, 1);
    for ind_ratio = 1:1:length(r)
        num_train = floor(num_train_all * r(ind_ratio));
        fprintf('*********\nUse training size:%d\n', num_train);
        train_data.V = slider_velocities_train(1:num_train, :);
        train_data.F = push_wrenches_train(1:num_train, :);
        % Evaluate on experimental test data.
        [record_exp] = MethodComparision(train_data, validation_data, testexp_data);
        for ind_method = 1:1:num_methods
            exp_record.err_test{ind_method}(ind_eval, ind_ratio) = record_exp.err_test(ind_method);
            exp_record.err_train{ind_method}(ind_eval, ind_ratio) = record_exp.err_train(ind_method);
            exp_record.err_validation{ind_method}(ind_eval, ind_ratio) = record_exp.err_validation(ind_method);
        end
        % Evaluate on simulation test data.
        [record_sim] = MethodComparision(train_data, validation_data, testsim_data);
        for ind_method = 1:1:num_methods
            sim_record.err_test{ind_method}(ind_eval, ind_ratio) = record_sim.err_test(ind_method);
            sim_record.err_train{ind_method}(ind_eval, ind_ratio) = record_sim.err_train(ind_method);
            sim_record.err_validation{ind_method}(ind_eval, ind_ratio) = record_sim.err_validation(ind_method);
        end
        
    end

end
toc;
group_name = {'10', '20', '30'};

data_exp = exp_record.err_test;
h_exp = PlotBarsWithErrors(data_exp, group_name);

data_sim = sim_record.err_test;
h_sim = PlotBarsWithErrors(data_sim, group_name);

% Plot error bars. 
%PlotTestTrainErrorBar(r, exp_record);
%PlotTestTrainErrorBar(r, sim_record);
