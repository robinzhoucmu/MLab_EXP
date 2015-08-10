clear all; close all;
log_file_name = 'SensorLogs/exp10_15.txt';
unit_scale = 1000;
H_tf = eye(4,4);
trans = [50;50;0];
H_tf = [eye(3,3), trans;
      0,0,0,1];
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';

% Parameters for trianglular block.         
Tri_mass = 1.518;
Tri_com = [0.15/3; 0.15/3];

% support Points
Tri_pts = [0.03    0.09    0.03;
           0.03    0.03    0.09];

%[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, Tri_com, Tri_mass);
Tri_pts = bsxfun(@minus, Tri_pts, Tri_com);
[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, [0;0], Tri_mass);

Tri_pho = 0.1;
%rng(1);

[pre_push_poses, post_push_poses, ft_readings, robot_pose_readings] = ParseLog(log_file_name);
num_pushes = size(pre_push_poses, 2);

push_wrenches = zeros(num_pushes, 3);
slider_velocities = zeros(num_pushes, 3);
slider_vel_raw = zeros(num_pushes, 3);
%i=2;
%num_pushes = 15;
for i = 1:1:num_pushes
    % Get object pre and post push pose.
    obj_start_pos = pre_push_poses(:,i);
    obj_end_pos = post_push_poses(:,i);
    % Use force reading time as timeline.
    t = ft_readings{i}(1, :);
    t0 = t(1);
    bsxfun(@minus, t, t0);
    force = ft_readings{i}(2:3, :);
    % Force torque sensor positive x is in the opposite direction of global
    % robot frame.
    %force(1,:) = - force(1,:);
    force = R_tool * force;
    % Negate sign to get force applied on the object.
    force = - force;
    % Minus offset. 
    force = bsxfun(@minus, force, force(:,1));
    %figure, plot(t, force');
    
    %Eliminate forces before the jump of the signal.
    avg_f = mean(force,2);
    index_small = sum(force.^2,1) < sum(avg_f.^2);
    %force(:, index_small) = [];
    %t(index_small) = [];
    
    % Hack: only use 25%-100% of the force signal.
    %index_pre_touch = 1:ceil(length(t) * 0.25);
    starting_p = 0.3;
    index_pre_touch = 1:length(t) < ceil(length(t) * starting_p);
    index_rm = index_pre_touch | index_small;
    force(:, index_rm) = [];
    t(index_rm) = [];
    %hold on; plot(t, force', 'y-*');
    
    N = length(t); 
    
    % Interpolate (linear) object poses. 
    [ obj_2d_traj ] = LinearInterpObjPos(obj_start_pos', obj_end_pos', N, unit_scale, H_tf);
    
    % Get robot trajectory. 
    robot_traj_0 = robot_pose_readings{i}(2:end, :);
    t_robot = robot_pose_readings{i}(1,:);
    bsxfun(@minus, t_robot, t0);
    % Align the first and last value of t_robot and t to be the same for
    % interpolation purpose.
    %t_robot(end) = t(end);
    if (t_robot(1) > t(1))
    %    t_robot(1) = t(1);
    end
    % Linear interpolate robot_pose.
    robot_traj = interp1(t_robot,robot_traj_0',t, 'linear','extrap')';
    robot_2d_pos = get2dPos(robot_traj',eye(4,4), unit_scale);
    % Convert into meters.
    %robot_traj = robot_traj / unit_scale;
    
    % Compute wrench in object local coordinate.
    wrench = ComputeWrench(obj_2d_traj, force', robot_2d_pos);
    % Minus offset. 
    %wrench = bsxfun(@minus, wrench, wrench(1,:));
    wrench(:,3) = wrench(:,3) / Tri_pho;
    
    vd_file = strcat('videos/out', int2str(i), '.avi');    
    % Trajectory video generation.
    %finger_2d_traj = robot_traj(1:2,:)'; [h] = check2dtraj_visualize(vd_file, obj_start_pos', obj_end_pos', finger_2d_traj/unit_scale);
    
    % Compute average wrench as the pushing force and normalized slider velocity. 
    push_wrenches(i,:) = mean(wrench);
    d = obj_2d_traj(end,:) - obj_2d_traj(1,:);
    % Rotate to initial object frame.
    theta = obj_2d_traj(1,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    d(1:2) = (R' * d(1:2)')';
    slider_vel_raw(i,:) = d / norm(d);
    d(3) = d(3) * Tri_pho;
    slider_velocities(i,:) = d / norm(d);
end
push_wrenches_dir = bsxfun(@rdivide, push_wrenches, sqrt(sum(push_wrenches.^2, 2)));

% Split training and testing data.
ratio_train = 0.9
NDataTrain = floor(num_pushes * ratio_train);
index_perm = randperm(num_pushes);

% Split train, test data.
push_wrenches_train = push_wrenches(index_perm(1:NDataTrain), :);
push_wrenches_test = push_wrenches(index_perm(NDataTrain+1:end), :);
push_wrenches_dir_train = push_wrenches_dir(index_perm(1:NDataTrain), :);
push_wrenches_dir_test = push_wrenches_dir(index_perm(NDataTrain+1:end), :);

slider_velocities_train = slider_velocities(index_perm(1:NDataTrain), :);
slider_velocities_test = slider_velocities(index_perm(NDataTrain+1:end), :);

% Fit limit surface.
flag_convex = 1;
w_force = 1;
w_vel = 1;
w_reg = 10;
[coeffs, xi, delta, pred_v_train, s] = Fit4thOrderPolyCVX(push_wrenches_train', slider_velocities_train', w_reg, w_vel, w_force, flag_convex);

%Linear Prediction (Quadratic fitting) baseline.
w_force_qp = 1;
w_reg_qp = 10;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(push_wrenches_train', slider_velocities_train', w_force_qp, w_reg_qp, flag_convex);

% Evaluate on training. 
%[pred_vel_dir_train] = GetVelFrom4thOrderPoly(coeffs, push_wrenches_train');
% Compute mean deviation angle.
% angles_train = acos(diag(slider_velocities_train * pred_vel_dir_train')) * 180 / pi;
% disp('Mean Train Angle(Degree) Deviation');
% mean(angles_train)

disp('Mean Train Angle(Degree) Deviation');
[err_poly ,dev_angle_poly] = EvaluatePoly4Predictor(push_wrenches_dir_train, slider_velocities_train, coeffs)

% Predict test data velocity and evaluate performance.
disp('Mean Test Angle(Degree) Deviation');
[err_poly ,dev_angle_poly] = EvaluatePoly4Predictor(push_wrenches_dir_test, slider_velocities_test, coeffs)

% [pred_vel_dir_test] = GetVelFrom4thOrderPoly(coeffs, push_wrenches_test');
% angles_test = acos(diag(slider_velocities_test * pred_vel_dir_test')) * 180 / pi;
% disp('Mean Test Angle(Degree) Deviation');
% mean(angles_test)

disp('Mean train error Linear');
[err, dev_angle] = EvaluateLinearPredictor(push_wrenches_dir_train, slider_velocities_train, A)
disp('Mean test error Linear');
[err, dev_angle] = EvaluateLinearPredictor(push_wrenches_dir_test, slider_velocities_test, A)

% Combine with symmetric data for training GP.

push_wrenches_dir_train_gp = [push_wrenches_dir_train; -push_wrenches_dir_train];

slider_velocities_train_gp = [slider_velocities_train; -slider_velocities_train];
[hyp, err_angle_train, err_angle_test] = GP_Fitting(push_wrenches_dir_train_gp, slider_velocities_train_gp, push_wrenches_dir_test, slider_velocities_test);
err_angle_test
err_angle_train


% Sample from the ideal pressure distribution and evaluate how good each
% predictor is. 
Nc = 200;
CORs = GenerateRandomCORs(Tri_pts, Nc, 300);
[F, bv] = GenFVPairsFromPD(Tri_pts, Tri_pds, CORs);
pho=Tri_pho;
F(3,:) = F(3,:) / pho;
bv(:,3) = bv(:,3) * pho;
bv = bsxfun(@rdivide, bv, sqrt(sum(bv.^2,2)));
F_dir = bsxfun(@rdivide, F, sqrt(sum(F.^2,1)));
[hyp, err_gp_train, dev_angle_gp] = GP_Fitting(push_wrenches_dir_train_gp, slider_velocities_train_gp, F_dir', bv)
[err_poly4,dev_angle_poly4] = EvaluatePoly4Predictor(F_dir', bv, coeffs)
[err_linear,dev_angle_linear] = EvaluateLinearPredictor(F_dir', bv, A)


 