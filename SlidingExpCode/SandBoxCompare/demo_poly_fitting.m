 clear all;
 split_ratio = 0.01;
 filename = 'push_log_4.mat';
 [Loads, Vel, Loads_Train, Dir_Vel_Train, Loads_Test, Dir_Vel_Test, Dir_Loads_Train, Dir_Loads_Test] = ...
    LoadData(filename, split_ratio);
% load('push_log_4.mat');
% 
% % Normalize velocities to get unit directions.
% Dir_Vel = bsxfun(@rdivide, Vel, sqrt(sum(Vel.^2,2)));
% Dir_Loads = bsxfun(@rdivide, Loads, sqrt(sum(Loads.^2,2)));
% 
% %Split Train, Test data.
% NData = size(Dir_Vel, 1);
% ratio_train = 0.1;
% NDataTrain = floor(NData * ratio_train);
% index_perm = randperm(NData);
% Loads_Train = Loads(index_perm(1:NDataTrain), :);
% Dir_Loads_Train = Dir_Loads(index_perm(1:NDataTrain), :);
% Loads_Test = Loads(index_perm(NDataTrain+1:end), :);
% Dir_Loads_Test = Dir_Loads(index_perm(NDataTrain+1:end), :);
% Dir_Vel_Train = Dir_Vel(index_perm(1:NDataTrain), :);
% Dir_Vel_Test = Dir_Vel(index_perm(NDataTrain+1:end), :);

% Fit limit surface with 4th order homogenious polynomial.
w_force = 1;
w_vel = 1;
w_reg = 0;
[coeffs, Q, xi, delta, pred_v_train, s] = Fit4thOrderPolyCVX(-Loads_Train', Dir_Vel_Train', w_reg, w_vel, w_force);
% Evaluate on training set. 
[pred_vel_dir_train] = GetVelFrom4thOrderPoly(coeffs, -Loads_Train');
angles_train = acos(diag(Dir_Vel_Train * pred_vel_dir_train')) * 180 / pi;
disp('Mean Training Angle(Degree) Deviation');
mean(angles_train)

% Predict test data velocity and evaluate performance.
[pred_vel_dir_test] = GetVelFrom4thOrderPoly(coeffs, -Dir_Loads_Test');
angles_test = acos(diag(Dir_Vel_Test * pred_vel_dir_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)

%Linear Prediction (Quadratic fitting) baseline.
lambda = 1; gamma = 0;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(-Loads_Train', Dir_Vel_Train', lambda, gamma);

disp('Mean Test Angle Deviation Linear Baseline');
[err, dev_angle] = EvaluateLinearPredictor(Dir_Vel_Test, Dir_Vel_Test, A)
