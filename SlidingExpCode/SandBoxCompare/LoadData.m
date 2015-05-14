function [Loads, Vel, Loads_Train, Dir_Vel_Train, Loads_Test, Dir_Vel_Test, Dir_Loads_Train, Dir_Loads_Test] = ...
    LoadData(filename, split_ratio)

load(filename);

% Normalize velocities to get unit directions.
Dir_Vel = bsxfun(@rdivide, Vel, sqrt(sum(Vel.^2,2)));
Dir_Loads = bsxfun(@rdivide, Loads, sqrt(sum(Loads.^2,2)));

%Split Train, Test data.
NData = size(Dir_Vel, 1);
ratio_train = split_ratio;
NDataTrain = floor(NData * ratio_train);
index_perm = randperm(NData);
Loads_Train = Loads(index_perm(1:NDataTrain), :);
Dir_Loads_Train = Dir_Loads(index_perm(1:NDataTrain), :);
Loads_Test = Loads(index_perm(NDataTrain+1:end), :);
Dir_Loads_Test = Dir_Loads(index_perm(NDataTrain+1:end), :);
Dir_Vel_Train = Dir_Vel(index_perm(1:NDataTrain), :);
Dir_Vel_Test = Dir_Vel(index_perm(NDataTrain+1:end), :);

end

