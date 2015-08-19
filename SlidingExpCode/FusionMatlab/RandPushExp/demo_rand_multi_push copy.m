clear all; close all;
%log_file_name = 'SensorLogs/10_90_10_10_30_130/exp_08_15_50.txt';
%log_file_name = 'SensorLogs/exp_08_11_50_mixed.txt';
%log_file_name = 'SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/wood_10_130_10_10_10_130/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/wood_patch/exp_08_17_0839_50.txt';
log_file_name = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
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
Tri_pts = [0.03    0.09    0.03; 0.03    0.03    0.09];
%Tri_pts = [0.01, 0.09,0.01;0.01,0.03,0.13];
%Tri_pts = [0.03, 0.06333, 0.04333; 0.03, 0.04333, 0.06333];
%Tri_pts = [0.01, 0.01, 0.13; 0.01,0.13,0.01];
%[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, Tri_com, Tri_mass);
Tri_pts = bsxfun(@minus, Tri_pts, Tri_com);
[Tri_pds, Tri_pho] = GetObjParaFromSupportPts(Tri_pts, [0;0], Tri_pressure);

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
ratio_train = 0.75;
[slider_velocities_train, slider_velocities_test, push_wrenches_train, push_wrenches_test] = ...
    SplitTrainTestData(slider_velocities, push_wrenches, ratio_train);
push_wrenches_dir_train = UnitNormalize(push_wrenches_train);
push_wrenches_dir_test = UnitNormalize(push_wrenches_test);

options_poly4.flag_convex = 1;
options_poly4.method = 'poly4';
options_poly4.flag_dir = 0;
[para_poly4] = CrossValidationSearchParameters(push_wrenches_train, slider_velocities_train, push_wrenches_test, slider_velocities_test, options_poly4)
h_poly4 = Plot4thPoly(para_poly4.coeffs, push_wrenches_train);
VisualizeForceVelPairs(push_wrenches_train', slider_velocities_train', h_poly4);

% Add non-convex constrained poly5.
options_poly4.flag_convex = 0;
options_poly4.method = 'poly4';
options_poly4.flag_dir = 0;
[para_poly4_plain] = CrossValidationSearchParameters(push_wrenches_train, slider_velocities_train, push_wrenches_test, slider_velocities_test, options_poly4)
h_poly4_plain = Plot4thPoly(para_poly4_plain.coeffs, push_wrenches_train);
VisualizeForceVelPairs(push_wrenches_train', slider_velocities_train', h_poly4_plain);

options_quadratic.flag_convex = 1;
options_quadratic.method = 'quadratic';
options_quadratic.flag_dir = 0;
[para_quadratic] = CrossValidationSearchParameters(push_wrenches_train, slider_velocities_train, push_wrenches_test, slider_velocities_test, options_quadratic)
A = para_quadratic.coeffs;
r = [A(1,1), A(2,2), A(3,3), A(1,2)*2, A(1,3)*2, A(2,3)*2];
h_quadratic = DrawEllipsoid(r, push_wrenches_dir_train);
VisualizeForceVelPairs(push_wrenches_train', slider_velocities_train', h_quadratic);

options_gp.method = 'gp';
[para_gp] = CrossValidationSearchParameters(push_wrenches_train, slider_velocities_train, push_wrenches_test, slider_velocities_test, options_gp)

% Combine with symmetric data for training GP.
 push_wrenches_dir_train_gp = [push_wrenches_dir_train; -push_wrenches_dir_train];
 slider_velocities_train_gp = [slider_velocities_train; -slider_velocities_train];
% disp('GP train and test');
% [hyp, err_angle_train, err_angle_test] = ...
%     GP_Fitting(push_wrenches_dir_train_gp, slider_velocities_train_gp, push_wrenches_dir_test, slider_velocities_test)


% Sample from the ideal pressure distribution and evaluate how good each
% predictor is. 
Nc = 402;
CORs = GenerateRandomCORs3(Tri_pts, Nc, 402/3);
[F, bv] = GenFVPairsFromPD(Tri_pts, Tri_pds, CORs);
% Change to row representation.
F = F';
pho=Tri_pho;
[bv, F] = NormalizeForceAndVelocities(bv, F, pho);
F_dir = UnitNormalize(F);

figure; plot3(F(:,1), F(:,2), F(:,3), '.');

figure;
k = convhull(F(:,1)', F(:,2)', F(:,3)');
trimesh(k, F(:,1)', F(:,2)', F(:,3)');

[err_gp, dev_angle_gp_train, dev_angle_gp] = GP_Fitting(push_wrenches_dir_train_gp, slider_velocities_train_gp, F_dir, bv, para_gp.coeffs)
[err_poly4,dev_angle_poly4] = EvaluatePoly4Predictor(F_dir, bv, para_poly4.coeffs)
[err_poly4_plain,dev_angle_poly4_plain] = EvaluatePoly4Predictor(F_dir, bv, para_poly4_plain.coeffs)
[err_linear,dev_angle_linear] = EvaluateLinearPredictor(F_dir, bv, para_quadratic.coeffs)


 