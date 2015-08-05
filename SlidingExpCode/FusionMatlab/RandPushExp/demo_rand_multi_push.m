log_file_name = 'exp0.txt';
unit_scale = 1000;
H_tf = eye(4,4);
Tri_pho = 0.1;

[pre_push_poses, post_push_poses, ft_readings, robot_pose_readings] = ParseLog(log_file_name);
num_pushes = size(pre_push_poses, 2);
i=2;
%for i = 1:1:num_pushes
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
    force(1,:) = - force(1,:);
    % Negate sign to get force applied on the object.
    force = - force;
    % Minus offset. 
    force = bsxfun(@minus, force, force(:,1));
    % Eliminate forces before the jump of the signal.
    avg_f = mean(force,2);
    index_small = sum(force.^2,1) < sum(avg_f.^2);
    force(:, index_small) = [];
    t(index_small) = [];
    
    N = length(t);
    
    
    % Interpolate (linear) object poses. 
    [ obj_2d_traj ] = LinearInterpObjPos(obj_start_pos', obj_end_pos', N, unit_scale, H_tf);
    
    % Get robot trajectory. 
    robot_traj_0 = robot_pose_readings{i}(2:end, :);
    t_robot = robot_pose_readings{i}(1,:);
    bsxfun(@minus, t_robot, t0);
    % Align the first and last value of t_robot and t to be the same for
    % interpolation purpose.
    t_robot(end) = t(end);
    t_robot(1) = t(1);
    % Linear interpolate robot_pose.
    robot_traj = interp1(t_robot,robot_traj_0',t)';
    % Convert into meters.
    robot_traj = robot_traj / unit_scale;
    
    % Compute wrench in object local coordinate.
    wrench = ComputeWrench(obj_2d_traj, force', robot_traj(1:2,:)');
    % Minus offset. 
    %wrench = bsxfun(@minus, wrench, wrench(1,:));
    wrench(:,3) = wrench(:,3) / Tri_pho;
    plot3curves(wrench, t);
    
    
    
    
    
    % Trajectory video generation.
    finger_2d_traj = robot_traj(1:2,:)'; [h] = check2dtraj_visualize('out.avi', obj_start_pos', obj_end_pos', finger_2d_traj);
%end
 