function [data, time] = read_from_log(mocap_file)
raw_data = dlmread(mocap_file);
time = raw_data(1:2:end,1);
data = raw_data(2:2:end,:);
end

