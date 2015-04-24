close all;
mocap_file = '../mocap_pos.txt';
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


