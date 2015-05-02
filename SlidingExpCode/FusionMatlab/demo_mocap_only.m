clear all;
close all;
mocap_file = '~/MLab_EXP/Mocap/PythonExp/mocap_pos.txt';

H_mb_lo = [0.230127 0.973003 0.00328255 -36.9078 
0.0269014 -0.0247022 0.999603 -33.5727 
0.972686 -0.229519 -0.0284225 -26.1142 
0 0 0 1 ];

[cart_pos, time] = read_cart_pos_log(mocap_file);
% Downsampling.
cart_pos = cart_pos(1:5:end,:);
time = time(1:5:end,:);
% cart_pos(176:183,:) = [];
% time(176:183,:) = [];

[pos_2d] = get2dPos(cart_pos, H_mb_lo);

figure, plot(time, pos_2d(:,1), 'r-'); hold on;  plot(time, pos_2d(:,2), 'g-'); plot(time, pos_2d(:,3), 'b-');

wsize1 = 10;
pos_2d_f = window_filter(pos_2d, wsize1);
time_f = time(wsize1:end);
figure, plot(time_f, pos_2d_f(:,1), 'r-'); hold on;  plot(time_f, pos_2d_f(:,2), 'g-'); plot(time_f, pos_2d_f(:,3), 'b-');

[V, A] = getLocalVelAccFromGlobalPose(pos_2d_f, time_f);
%V = bsxfun(@rdivide, V, max(abs(V)));
%A = bsxfun(@rdivide, A, max(abs(A)));

wsize2 = 4;
V = window_filter(V, wsize2);
A = window_filter(A, wsize2);

ind_moving = sum(V.^2, 2) > 0.01;
V_m = V(ind_moving,:);
A_m = A(ind_moving,:);

t_m = time_f(ind_moving);
V_m_dir = bsxfun(@rdivide, V_m, sqrt(sum(V_m.^2,2)));
A_m_dir = bsxfun(@rdivide, A_m, sqrt(sum(A_m.^2,2)));

%plot3curves(V_m_dir, t_m);
%plot3curves(A_m_dir, t_m);


[coeffs, Q, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(A_m_dir', V_m_dir', 0, 100, 1);
[dir_vel] = GetVelFrom4thOrderPoly(coeffs, A_m');
disp('mean predicted error');
err_vel = dir_vel - V_m_dir;
mean(sqrt(sum(err_vel.^2,2)))
plot3curves(dir_vel, t_m);

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

