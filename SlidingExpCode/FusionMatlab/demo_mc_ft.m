clear all;
close all;

% Parameters for trianglular block.         
Tri_mass = 0.128;
Tri_com = [0.0293; 0.0293];
% Close to center.
% Tri_pts = [0.0280    0.0480    0.0280
%        0.0280    0.0280    0.0480];

% Spread out.
Tri_pts = [0.01    0.084    0.01;
           0.01    0.01    0.084];

[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, Tri_com, Tri_mass);

%Tri_pho = 0.1;

% Instrinsic calibration matrix of the block.
H_mc_obj = [-0.582574 0.812668 -0.00531035 -2.38805 
0.00730846 0.0198042 0.999895 -35.7798 
0.812695 0.582376 -0.0127986 -39.5608 
0 0 0 1 ];

        
dir_path = '~/MLab_EXP/Mocap/PythonExp/exp4/';
[obj_cart, t_obj] = read_from_log(strcat(dir_path,'pos3.txt'));
[robot_cart, t_robot] = read_from_log(strcat(dir_path, 'robot3.txt'));
[force, t_force] = read_from_log(strcat(dir_path, 'force3.txt'));
% Delete repetitive time.
ind_time_still = find((t_robot(2:end) - t_robot(1:end-1)) == 0);
robot_cart(ind_time_still, :) = [];
t_robot(ind_time_still, :) = [];
 
%obj_cart(:,3) = 45;
ind_neg_quat = obj_cart(:,4) < 0;
obj_cart(ind_neg_quat,4:end) = - obj_cart(ind_neg_quat, 4:end);


% FT sensor +fx is the same as tool frame +x, which is negative of robot
% frame +x.
force(:,1) = -force(:,1);

% Subtract initial force (mean value of the first 5 readings).
force = bsxfun(@minus, force, mean(force(1:5,:)));

% Negate sign to get pushing force.
force = -force;

% Convert robot reading from mm to meters.
unit_scale = 1000;
robot_cart(:,1:3) = robot_cart(:,1:3) / unit_scale;

[fused_obj_cart, fused_robot_cart, fused_force, t_ref] = sensorFusionByTime(obj_cart, t_obj, robot_cart, t_robot, force, t_force);
t_ref = bsxfun(@minus, t_ref, t_ref(1));

% Subsample the signal.
sample_interval = 1;
t_ref = t_ref(1:sample_interval:end);
fused_obj_cart = fused_obj_cart(1:sample_interval:end,:);
fused_robot_cart = fused_robot_cart(1:sample_interval:end,:);
fused_force = fused_force(1:sample_interval:end,:);

%----------------------------------------------
%----------------------------------------------

% Return 2d pose in meters, radian.
%obj_pos_2d = get2dPos(fused_obj_cart, H_mc_obj, unit_scale);

% Convert force to robot frame.
%fused_wrench = ComputeWrench(obj_pos_2d, fused_force(:,1:2), fused_robot_cart(:,1:2));

% Filtering.
t_cp = t_ref;
NOrder = 50;
CutFreq = 1;
%[obj_pos_2d, t_ref] = FIRFilter(obj_pos_2d, t_cp, NOrder, CutFreq);
[fused_obj_cart2, t_ref] = FIRFilter(fused_obj_cart, t_cp, NOrder, CutFreq);
%[fused_wrench, t_ref] = FIRFilter(fused_wrench, t_cp, NOrder, CutFreq);
[fused_force2, t_ref] = FIRFilter(fused_force, t_cp, NOrder, CutFreq);

fused_robot_cart = interp1(t_cp,fused_robot_cart,t_ref);

% Return 2d pose in meters, radian.
obj_pos_2d = get2dPos(fused_obj_cart2, H_mc_obj, unit_scale);

% Convert force to robot frame.
fused_wrench = ComputeWrench(obj_pos_2d, fused_force2(:,1:2), fused_robot_cart(:,1:2));

%[obj_pos_2d, t_ref] = FIRFilter(obj_pos_2d, t_ref, NOrder, CutFreq);

% Assume robot moves in a plane. The finger would be the x,y part. 
robot_finger = fused_robot_cart(:, 1:2);

% Compute velocities (mid-value difference)
[Vel,Acc] = getLocalVelAccFromGlobalPose(obj_pos_2d, t_ref);

% Extract Forces for corresponding time.
Loads = fused_wrench(2:end-1,:);

% Get robot pushing direction and contact points in local object frame.
[VelPush, PtContact] = getLocalPushVelDir(obj_pos_2d, robot_finger, t_ref);
VelPush = VelPush(2:end,:);
PtContact = PtContact(2:end, :);


% Erdman's normalization.
Vel(:,3) = Vel(:,3) * Tri_pho;
Loads(:,3) = Loads(:,3) / Tri_pho;

figure;
plot3(Loads(:,1), Loads(:,2), Loads(:,3), 'r*', 'Markersize', 6);
title('Load Scatter Plot');
plot3curves(Vel, t_ref(2:end-1));
title('Velocity plot');
plot3curves(Loads, t_ref(2:end-1));
title('Load plot');

% Remove static parts.
eps_moving = 0.0005;  
eps_toofast = 0.005;
eps_cf = 0.1;
eps_robot_moving = 1e-3;
ind_speedfilter = sqrt(sum(Vel.^2,2)) < eps_moving | ...
                  sqrt(sum(Vel.^2,2)) > eps_toofast | ...
                  sqrt(sum(Loads.^2,2)) < eps_cf | ...
                  sqrt(sum(VelPush.^2,2)) < eps_robot_moving; 

Vel(ind_speedfilter,:) = [];
Loads(ind_speedfilter,:) = [];
t_ref_moving = t_ref(~ind_speedfilter);

VelPush(ind_speedfilter,:) = []; PtContact(ind_speedfilter, :) = [];
VelPushNormalized = bsxfun(@rdivide, VelPush, sqrt(sum(VelPush.^2,2)));

figure;
plot3(Loads(:,1), Loads(:,2), Loads(:,3), 'r*', 'Markersize', 6);
title('Load Scatter Plot');
plot3curves(Vel, t_ref_moving);
title('Velocity plot');
plot3curves(Loads, t_ref_moving);
title('Load plot');
% Normalize velocities to get directions.
Dir_Vel = bsxfun(@rdivide, Vel, sqrt(sum(Vel.^2,2)));
Dir_Loads = bsxfun(@rdivide, Loads, sqrt(sum(Loads.^2,2)));

NData = size(Dir_Vel, 1);
ratio_train = 0.5;
NDataTrain = floor(NData * ratio_train);
index_perm = randperm(NData);

%Split Train, Test data.
Loads_Train = Loads(index_perm(1:NDataTrain), :);
Dir_Loads_Train = Dir_Loads(index_perm(1:NDataTrain), :);

Loads_Test = Loads(index_perm(NDataTrain+1:end), :);
Dir_Loads_Test = Dir_Loads(index_perm(NDataTrain+1:end), :);

Dir_Vel_Train = Dir_Vel(index_perm(1:NDataTrain), :);
Dir_Vel_Test = Dir_Vel(index_perm(NDataTrain+1:end), :);

std_Loads = std(Loads)
std_Dir_Vel = std(Dir_Vel)


VisualizeForceVelPairs(Loads', Dir_Vel')

% Fit limit surface.
flag_convex = 1;
w_force = 0;
w_vel = 1;
w_reg = 0.1;
[coeffs, xi, delta, pred_v_train, s] = Fit4thOrderPolyCVX(Loads_Train', Dir_Vel_Train', w_reg, w_vel, w_force, flag_convex);

%Linear Prediction (Quadratic fitting) baseline.
%lambda = 1; gamma = 0;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(Loads_Train', Dir_Vel_Train', w_force, w_reg, flag_convex);

% Evaluate on training. 
[pred_vel_dir_train] = GetVelFrom4thOrderPoly(coeffs, Dir_Loads_Train');
% Compute mean deviation angle.
angles_train = acos(diag(Dir_Vel_Train * pred_vel_dir_train')) * 180 / pi;
disp('Mean Train Angle(Degree) Deviation');
mean(angles_train)

% Predict test data velocity and evaluate performance.
[pred_vel_dir_test] = GetVelFrom4thOrderPoly(coeffs, Dir_Loads_Test');
angles_test = acos(diag(Dir_Vel_Test * pred_vel_dir_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)


disp('Mean test error Linear');
[err, dev_angle] = EvaluateLinearPredictor(Dir_Loads_Test, Dir_Vel_Test, A)

[hyp, err_angle_train, err_angle_test] = GP_Fitting(Dir_Loads_Train, Dir_Vel_Train, Dir_Loads_Test, Dir_Vel_Test);
err_angle_test
err_angle_train

