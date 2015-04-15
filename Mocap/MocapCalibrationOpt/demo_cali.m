close all;
% Give initialization from physical measurement.
v0 = [0; 0; 200];
file_name = '../mocap_log.txt';
[RotMats, ToolPts, MocapPts] = ReadFromRosOutput(file_name);
N = size(ToolPts, 2);
% Use first 90% for fitting.
ind = ceil(N * 0.9);
[R_est t_est v_est err_est] = ...
    AlterDescent(ToolPts(:,1:ind), MocapPts(:,1:ind), RotMats(1:ind), v0);
% Check for validation error.
%meanErr = ValidFitting(ToolPts(:,ind+1:N), MocapPts(:,ind+1:N), RotMats(ind+1:N), R_est, t_est, v_est)

% Remove outlier.
threshold = 20;
[ToolPts, MocapPts, RotMats] = ...
    RemoveOutliers(ToolPts, MocapPts, RotMats, R_est, t_est, v_est, threshold);


% Use all data to fit final result and output to disk.
[R_final t_final v_final err_final] = AlterDescent(ToolPts, MocapPts, RotMats, v_est)
ValidFitting(ToolPts, MocapPts, RotMats, R_final, t_final, v_final)

output_file_name = '../matlab_cali_result.txt';
quat = qGetQ(R_final);
T = [t_final; quat]';
dlmwrite(output_file_name, T, ' ');
