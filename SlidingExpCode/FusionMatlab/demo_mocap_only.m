clear all;
close all;


H_mb_lo = [0.230127 0.973003 0.00328255 -36.9078 
0.0269014 -0.0247022 0.999603 -33.5727
0.972686 -0.229519 -0.0284225 -26.1142 
0 0 0 1 ];

[obj_cart, t_ref] = read_from_log('~/MLab_EXP/Mocap/PythonExp/mocap_pos_5.txt');
t_ref = t_ref - t_ref(1);
unit_scale = 1000.0;
% Return 2d pose in meters, radian.
obj_pos_2d = get2dPos(obj_cart, H_mb_lo, unit_scale);

% Erdmann's normalization.
pho = 0.05;
obj_pos_2d(:,3) = obj_pos_2d(:,3) * pho;
plot3curves(obj_pos_2d, t_ref);


% [Vel,Acc] = getLocalVelAccFromGlobalPose(obj_pos_2d, t_ref);
% plot3curves(Vel, t_ref(2:end-1));
% plot3curves(Acc, t_ref(2:end-1));


% windowSize = 20;
% sigma = 10;
% filter = fspecial('gaussian', [windowSize 1], sigma);
% obj_pos_2d_f2 = zeros(size(obj_pos_2d,1) + windowSize - 1, 3);
% for i = 1:1:3
%     obj_pos_2d_f2(:,i) = conv(obj_pos_2d(:,i), filter);
% end
% plot3curves(obj_pos_2d_f2(windowSize/2 : end-windowSize/2,:), t_ref);

t_cp = t_ref;
[obj_pos_2d_f, t_ref] = FIRFilter(obj_pos_2d, t_cp, 50, 1);
plot3curves(obj_pos_2d_f, t_ref);

% Subsample the signal.
sample_interval = 4;
t_ref = t_ref(1:sample_interval:end);
obj_pos_2d_f = obj_pos_2d_f(1:sample_interval:end,:);


% Compute velocities (mid-value difference)
[Vel_f,Acc_f] = getLocalVelAccFromGlobalPose(obj_pos_2d_f, t_ref);
plot3curves(Vel_f, t_ref(2:end-1));
plot3curves(Acc_f, t_ref(2:end-1));


% Remove accelerating parts. Check for kinetic energy.
Vel_m = Vel_f;
Acc_m = Acc_f;
t_ref_m = t_ref(2:end-1);

ke = sum(Vel_m.^2,2);
dke = diff(ke);
ind_acc = dke > 0.5;
%ind_acc = max(Acc_m, [], 2) > 0;
Vel_m(ind_acc,:) = [];
Acc_m(ind_acc,:) = [];
t_ref_m(ind_acc,:) = [];

% Remove static parts.
ind_static = sqrt(sum(Vel_m.^2,2)) < 0.1;
Vel_m(ind_static,:) = [];
Acc_m(ind_static,:) = [];
t_ref_m(ind_static,:) = [];

plot3curves(Vel_m, t_ref_m);
plot3curves(Acc_m, t_ref_m);

% Normalize velocities to get directions.
Dir_Vel_m = bsxfun(@rdivide, Vel_m, sqrt(sum(Vel_m.^2,2)));
Dir_Acc_m = bsxfun(@rdivide, Acc_m, sqrt(sum(Acc_m.^2,2)));


% Fit limit surface.
w_force = 0;
w_vel = 1;
w_reg = 0;
[coeffs, Q, xi, delta, pred_v_train, s] = Fit4thOrderPolyCVX(-Acc_m', Dir_Vel_m', w_reg, w_vel, w_force);

% Evaluate on training. 
[pred_vel_dir_train] = GetVelFrom4thOrderPoly(coeffs, -Dir_Acc_m');
% Compute mean deviation angle.
angles_train = acos(diag(Dir_Vel_m * pred_vel_dir_train')) * 180 / pi;
disp('Mean Train Angle(Degree) Deviation');
mean(angles_train)

% Linear Prediction (Quadratic fitting) baseline.
w_force = 0; w_reg = 0;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(-Dir_Acc_m', Dir_Vel_m', w_force, w_reg);

disp('Mean Error Linear');
[err, dev_angle] = EvaluateLinearPredictor(-Dir_Acc_m, Dir_Vel_m, A)


















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% mocap_file = '~/MLab_EXP/Mocap/PythonExp/mocap_pos.txt';
% 
% H_mb_lo = [0.230127 0.973003 0.00328255 -36.9078 
% 0.0269014 -0.0247022 0.999603 -33.5727 
% 0.972686 -0.229519 -0.0284225 -26.1142 
% 0 0 0 1 ];
% 
% [cart_pos, time] = read_cart_pos_log(mocap_file);
% % Downsampling.
% cart_pos = cart_pos(1:5:end,:);
% time = time(1:5:end,:);
% % cart_pos(176:183,:) = [];
% % time(176:183,:) = [];
% 
% [pos_2d] = get2dPos(cart_pos, H_mb_lo);
% 
% figure, plot(time, pos_2d(:,1), 'r-'); hold on;  plot(time, pos_2d(:,2), 'g-'); plot(time, pos_2d(:,3), 'b-');
% 
% wsize1 = 10;
% pos_2d_f = window_filter(pos_2d, wsize1);
% time_f = time(wsize1:end);
% figure, plot(time_f, pos_2d_f(:,1), 'r-'); hold on;  plot(time_f, pos_2d_f(:,2), 'g-'); plot(time_f, pos_2d_f(:,3), 'b-');
% 
% [V, A] = getLocalVelAccFromGlobalPose(pos_2d_f, time_f);
% %V = bsxfun(@rdivide, V, max(abs(V)));
% %A = bsxfun(@rdivide, A, max(abs(A)));
% 
% wsize2 = 4;
% V = window_filter(V, wsize2);
% A = window_filter(A, wsize2);
% 
% ind_moving = sum(V.^2, 2) > 0.01;
% V_m = V(ind_moving,:);
% A_m = A(ind_moving,:);
% 
% t_m = time_f(ind_moving);
% V_m_dir = bsxfun(@rdivide, V_m, sqrt(sum(V_m.^2,2)));
% A_m_dir = bsxfun(@rdivide, A_m, sqrt(sum(A_m.^2,2)));
% 
% %plot3curves(V_m_dir, t_m);
% %plot3curves(A_m_dir, t_m);
% 
% 
% [coeffs, Q, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(A_m_dir', V_m_dir', 0, 100, 1);
% [dir_vel] = GetVelFrom4thOrderPoly(coeffs, A_m');
% disp('mean predicted error');
% err_vel = dir_vel - V_m_dir;
% mean(sqrt(sum(err_vel.^2,2)))
% plot3curves(dir_vel, t_m);

% [v, t_v] = first_order_difference(pos_2d_f, time_f);
% 
% plot3curves(v, t_v);
% 
% % Filter out object-still snapshots.
% ind_still = sum(v.^2,2) < 0.01;
% v(ind_still, :) = [];
% t_v(ind_still, :) = [];
% % Filter velocity. 
% v = window_filter(v, wsize);
% t_v = t_v(wsize:end);
% 
% 
% [a, t_a] = first_order_difference(v, t_v);
% % Filter acceleration. 
% a = window_filter(a, wsize);
% t_a = t_a(wsize:end);
% 
% v = v(wsize+1:end-1, :);
% dir_a = bsxfun(@rdivide, a, sqrt(sum(a.^2,2)));
% dir_v = bsxfun(@rdivide, v, sqrt(sum(v.^2,2)));
% 
% plot3curves(dir_v, t_a);
% plot3curves(dir_a, t_a);
% 
% 
% [coeffs, Q, xi, delta, V, s] = Fit4thOrderPolyCVX(dir_a', dir_v', 0, 10000, 0);
% 
% [dir_vel] = GetVelFrom4thOrderPoly(coeffs, dir_a');
% plot3curves(dir_vel, t_a);

