function [RotMats, ToolPts, MocapPts] = ReadFromRosOutput(file_name)
M = dlmread(file_name);
N = size(M, 1) / 5;
% For every 5 lines, the first 3 correpsponds to rotation matrix pose.
%RotPoses = zeros(N, 3);
RotMats = cell(N,1);
ToolPts = zeros(3, N);
MocapPts = zeros(3, N);
for i = 1:1:N
    RotMats{i} = M(5*i-4:5*i-2, :);
    ToolPts(:,i) = M(5*i-1, :)';
    % Note that Mocap output in meters while robot output in milimeters.
    % MocapPts(:,i) = 1000 * M(5*i, :)';
    MocapPts(:,i) = M(5*i, :)';
end
end

