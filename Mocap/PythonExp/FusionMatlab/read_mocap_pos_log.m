function [cart_pos, time] = read_mocap_pos_log(mocap_file)
data = dlmread(mocap_file);
time = data(1:2:end,1);
% Change to elapsed time.
time = time - time(1);
cart_pos = data(2:2:end,:);
end

