close all;
% Give initialization from physical measurement.
%v0 = [0; 0; 200];
v0 = [0;0;260];
file_name = '../mocap_log.txt';
[RotMats, ToolPts, MocapPts] = ReadFromRosOutput(file_name);
N = size(ToolPts, 2);
% Use first 90% for fitting.
r = 0.8;
ind = floor(N * r);
ToolPts_Train = ToolPts(:,1:ind);
MocapPts_Train = MocapPts(:,1:ind);
RotMats_Train = RotMats(1:ind);

ToolPts_Test = ToolPts(:,ind+1:end);
MocapPts_Test = MocapPts(:,ind+1:end);
RotMats_Test = RotMats(ind+1:end);

[R_est t_est v_est err_est] = ...
    AlterDescent(ToolPts_Train, MocapPts_Train, RotMats_Train, v0);
% Check for error.
err_train_0 = ValidFitting(ToolPts_Train, MocapPts_Train, RotMats_Train, R_est, t_est, v_est) 
err_test_0 = ValidFitting(ToolPts_Test, MocapPts_Test, RotMats_Test, R_est, t_est, v_est)


% Remove outlier. Round 1;
threshold = 10;
[ToolPts_Train, MocapPts_Train, RotMats_Train] = ...
    RemoveOutliers(ToolPts_Train, MocapPts_Train, RotMats_Train, R_est, t_est, v_est, threshold);

% % Round 2 fitting.
% [R_est t_est v_est err_est] = ...
%     AlterDescent(ToolPts, MocapPts, RotMats, v_est);
% 
% % Round 2 removing outliers.
% threshold = 5;
% [ToolPts, MocapPts, RotMats] = ...
%     RemoveOutliers(ToolPts, MocapPts, RotMats, R_est, t_est, v_est, threshold);

% Use all data to fit final result and output to disk.
[R_final t_final v_final err_final] = AlterDescent(ToolPts_Train, MocapPts_Train, RotMats_Train, v_est)

err_train_1 = ValidFitting(ToolPts_Train, MocapPts_Train, RotMats_Train, R_final, t_final, v_final) 
err_test_1 = ValidFitting(ToolPts_Test, MocapPts_Test, RotMats_Test, R_final, t_final, v_final)

output_file_name = '../matlab_cali_result3.txt';
quat = qGetQ(R_final);
T = [t_final; quat]';
dlmwrite(output_file_name, T, ' ');
