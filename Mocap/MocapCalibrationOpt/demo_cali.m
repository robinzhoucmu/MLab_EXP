close all;
% Give initialization from physical measurement.
v0 = [0; 0; 200];
file_name = '../mocap_log.txt';
[RotMats, ToolPts, MocapPts] = ReadFromRosOutput(file_name);
N = size(ToolPts, 2);
% Use first 90% for fitting.
r = 1.0;
ind = floor(N * r);
[R_est t_est v_est err_est] = ...
    AlterDescent(ToolPts(:,1:ind), MocapPts(:,1:ind), RotMats(1:ind), v0);
% Check for error.
ValidFitting(ToolPts, MocapPts, RotMats, R_est, t_est, v_est)

% Remove outlier. Round 1;
threshold = 10;
[ToolPts, MocapPts, RotMats] = ...
    RemoveOutliers(ToolPts, MocapPts, RotMats, R_est, t_est, v_est, threshold);

% Round 2 fitting.
[R_est t_est v_est err_est] = ...
    AlterDescent(ToolPts, MocapPts, RotMats, v_est);

% Round 2 removing outliers.
threshold = 5;
[ToolPts, MocapPts, RotMats] = ...
    RemoveOutliers(ToolPts, MocapPts, RotMats, R_est, t_est, v_est, threshold);

% Use all data to fit final result and output to disk.
[R_final t_final v_final err_final] = AlterDescent(ToolPts, MocapPts, RotMats, v_est)

ValidFitting(ToolPts, MocapPts, RotMats, R_final, t_final, v_final)

output_file_name = '../matlab_cali_result3.txt';
quat = qGetQ(R_final);
T = [t_final; quat]';
dlmwrite(output_file_name, T, ' ');
