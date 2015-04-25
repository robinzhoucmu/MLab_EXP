clear all;
close all;
mocap_file = '../../Mocap/PythonExp/mocap_pos.txt';
H_mb_lo = [0.798368 -0.60081 -0.0583333 -5.71732 
           0.0495249 -0.0129369 0.998697 -34.4656 
           -0.600536 -0.799801 0.00481045 44.4191 
           0 0 0 1 ];
       
[cart_pos, time] = read_mocap_pos_log(mocap_file);
[pos_2d] = get2dPos(cart_pos, H_mb_lo);

figure, plot(time, pos_2d(:,1), 'r-'); hold on;  plot(time, pos_2d(:,2), 'g-'); plot(time, pos_2d(:,3), 'b-');

wsize = 6;
pos_2d_f = window_filter(pos_2d, wsize);
time_f = time(wsize:end);
figure, plot(time_f, pos_2d_f(:,1), 'r-'); hold on;  plot(time_f, pos_2d_f(:,2), 'g-'); plot(time_f, pos_2d_f(:,3), 'b-');

[v, t_v] = first_order_difference(pos_2d_f, time_f);

plot3curves(v, t_v);

% Filter out object-still snapshots.
ind_still = sum(v.^2,2) < 1;
v(ind_still, :) = [];
t_v(ind_still, :) = [];
% Filter velocity. 
v = window_filter(v, wsize);
t_v = t_v(wsize:end);

plot3curves(v, t_v);

[a, t_a] = first_order_difference(v, t_v);
% Filter acceleration. 
a = window_filter(a, wsize);
t_a = t_a(wsize:end);

plot3curves(a, t_a);

v = v(wsize+1:end-1, :);
dir_a = bsxfun(@rdivide, a, sqrt(sum(a.^2,2)));
dir_v = bsxfun(@rdivide, v, sqrt(sum(v.^2,2)));


