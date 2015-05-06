clear all;
close all;
%rng(1);

H_mb_lo = [0.230127 0.973003 0.00328255 -36.9078 
0.0269014 -0.0247022 0.999603 -33.5727
0.972686 -0.229519 -0.0284225 -26.1142 
0 0 0 1 ];

[obj_cart, t_obj] = read_from_log('~/MLab_EXP/Mocap/PythonExp/pos4.txt');
[robot_cart, t_robot] = read_from_log('~/MLab_EXP/Mocap/PythonExp/robot4.txt');
[force, t_force] = read_from_log('~/MLab_EXP/Mocap/PythonExp/force4.txt');
% FT sensor +fx is the same as tool frame +x, which is negative of robot
% frame +x.
force(:,1) = -force(:,1);

% Subtract initial force.
force = bsxfun(@minus, force, force(1,:));

% Convert robot reading from mm to meters.
robot_cart(:,1:3) = robot_cart(:,1:3) / 1000.0;

[fused_obj_cart, fused_robot_cart, fused_force, t_ref] = sensorFusionByTime(obj_cart, t_obj, robot_cart, t_robot, force, t_force);
t_ref = bsxfun(@minus, t_ref, t_ref(1));
% Return 2d pose in meters, radian.
obj_pos_2d = get2dPos(fused_obj_cart, H_mb_lo, 1000);

% Convert force to robot frame.
fused_wrench = ComputeWrench(obj_pos_2d, fused_force(:,1:2), fused_robot_cart(:,1:2));

%Filter the signal.
% alpha_new = 0.8;
% obj_pos_2d = EMA_Filter(obj_pos_2d, alpha_new);
% fused_wrench = EMA_Filter(fused_wrench, alpha_new);

% Window filter.
% windowSize = 20;
% b = (1/windowSize) * ones(1,windowSize);
% a = 1;
% obj_pos_2d = filter(b,a,obj_pos_2d);
% obj_pos_2d = obj_pos_2d(windowSize + 1:end,:);
% fused_wrench = filter(b,a,fused_wrench);
% fused_wrench = fused_wrench(windowSize+1:end,:);
% t_ref = t_ref(windowSize+1:end);
t_cp = t_ref;
[obj_pos_2d, t_ref] = FIRFilter(obj_pos_2d, t_cp);
[fused_wrench, t_ref] = FIRFilter(fused_wrench, t_cp);
fused_robot_cart = interp1(t_cp,fused_robot_cart,t_ref);

% Subsample the signal.
sample_interval = 2;
t_ref_sub = t_ref(1:sample_interval:end);
obj_pos_2d_sub = obj_pos_2d(1:sample_interval:end,:);
fused_robot_cart_sub = fused_robot_cart(1:sample_interval:end,:);
fused_wrench_sub = fused_wrench(1:sample_interval:end,:);

% Compute velocities (mid-value difference)
[Vel,Acc] = getLocalVelAccFromGlobalPose(obj_pos_2d_sub, t_ref_sub);
Vel_All = Vel;
Acc_All = Acc;

% Extract Forces for corresponding time.
Loads = fused_wrench_sub(2:end-1,:);
figure;
plot3(Loads(:,1), Loads(:,2), Loads(:,3), 'r*', 'Markersize', 6);title('Load Scatter Plot');
plot3curves(Vel, t_ref_sub(2:end-1));
title('Velocity plot');
plot3curves(Loads, t_ref_sub(2:end-1));
title('Load plot');

% Remove static parts.
ind_static = sqrt(sum(Vel.^2,2)) < 0.005;
Vel(ind_static,:) = [];
Loads(ind_static,:) = [];
t_ref_sub_moving = t_ref_sub(~ind_static);

body_length = 0.1;
Vel(:,1:2) = Vel(:,1:2) / body_length;
Vel(:,3) = Vel(:,3);
Loads(:,3) = Loads(:,3) / body_length;

% Normalize velocities to get directions.
Dir_Vel = bsxfun(@rdivide, Vel, sqrt(sum(Vel.^2,2)));
Dir_Loads = bsxfun(@rdivide, Loads, sqrt(sum(Loads.^2,2)));

NData = size(Dir_Vel, 1);
NDataTrain = floor(NData * 0.8);
index_perm = randperm(NData);
%Split Train, Test data.
Loads_Train = Loads(index_perm(1:NDataTrain), :);
Dir_Loads_Train = Dir_Loads(index_perm(1:NDataTrain), :);

Loads_Test = Loads(index_perm(NDataTrain+1:end), :);
Dir_Loads_Test = Dir_Loads(index_perm(NDataTrain+1:end), :);

Dir_Vel_Train = Dir_Vel(index_perm(1:NDataTrain), :);
Dir_Vel_Test = Dir_Vel(index_perm(NDataTrain+1:end), :);

% Fit limit surface.
w_force = 0.1;
w_vel = 1;
w_reg = 0.01;
[coeffs, Q, xi, delta, pred_v_train, s] = Fit4thOrderPolyCVX(-Loads_Train', Dir_Vel_Train', w_reg, w_vel, w_force);


%Linear Prediction (Quadratic fitting) baseline.
lambda = 100; gamma = 1000;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(-Loads_Train', Dir_Vel_Train', lambda, gamma);

% Evaluate on training. 
[pred_vel_dir_train] = GetVelFrom4thOrderPoly(coeffs, -Dir_Loads_Train');
% Compute mean deviation angle.
angles_train = acos(diag(Dir_Vel_Train * pred_vel_dir_train')) * 180 / pi;
disp('Mean Train Angle(Degree) Deviation');
mean(angles_train)

% Predict test data velocity and evaluate performance.
[pred_vel_dir_test] = GetVelFrom4thOrderPoly(coeffs, -Dir_Loads_Test');
angles_test = acos(diag(Dir_Vel_Test * pred_vel_dir_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)

disp('Mean test error Linear');
[err, dev_angle] = EvaluateLinearPredictor(Dir_Vel_Test, Dir_Vel_Test, A)


% avgVelTrain = mean(Dir_Vel_Train);
% disp('Just predict average value of train')
% mean(acos(diag(repmat(avgVelTrain, 18 , 1) * Dir_Vel_Test')) * 180 / pi)




