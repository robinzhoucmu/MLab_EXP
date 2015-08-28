function [push_actions] = ParsePushActionLog(log_file_name)
fid = fopen(log_file_name);
num_pushes = fscanf(fid, '%d', 1);
push_actions.push_vectors = zeros(3, num_pushes);
push_actions.push_points = zeros(3, num_pushes);
push_actions.approach_vectors = zeros(3, num_pushes);
push_actions.penetration_dist = zeros(num_pushes, 1);
push_actions.moveclose_dist = zeros(num_pushes, 1);

for ind = 1:1:num_pushes
    push_actions.type{ind} = fscanf(fid, '%s', 1);
    push_actions.push_points(1:3, ind) = fscanf(fid, '%f %f %f', [3 1]);
    push_actions.push_vectors(1:3, ind) = fscanf(fid, '%f %f %f', [3 1]);
    push_actions.approach_vectors(1:3, ind) = fscanf(fid, '%f %f %f', [3 1]);
    push_actions.penetration_dist(ind) = fscanf(fid, '%f', 1);
    push_actions.moveclose_dist(ind) = fscanf(fid, '%f', 1);
end
fclose(fid);
end

