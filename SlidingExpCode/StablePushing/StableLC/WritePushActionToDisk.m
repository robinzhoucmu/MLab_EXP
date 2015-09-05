function [selected_index] = WritePushActionToDisk(push_actions, file_name, flag_stable, num_samples)
fid = fopen(file_name, 'w');
num_pushes = size(push_actions.push_points,2);
if (nargin < 4)
    num_samples = num_pushes;
end
fprintf(fid, '%d\n', num_samples);
ct = 1;
ind_pointer = 0;
perm = randperm(num_pushes);
selected_index = zeros(num_samples, 1);
while (ct <= num_samples) && ind_pointer < num_pushes
    ind_pointer = ind_pointer + 1;
    ind = perm(ind_pointer);
    if (~flag_stable(ind))
        continue;
    end
    selected_index(ct) = ind; 
    fprintf(fid, '%s\n', push_actions.type{ind});
    fprintf(fid, '%f %f %f\n', push_actions.push_points(1:3, ind));
    flag_is_rot = strcmp(push_actions.type{ind}, 'TwoPointRotation');
    flag_is_trans = (strcmp(push_actions.type{ind}, 'TwoPointTranslation') || strcmp(push_actions.type{ind}, 'Point'));
    if flag_is_rot
        fprintf(fid, '%f %f %f\n', push_actions.cors(1:3, ind));
    elseif flag_is_trans
        fprintf(fid, '%f %f %f\n', push_actions.push_vectors(1:3, ind));
    end
    fprintf(fid, '%f %f %f\n', push_actions.approach_vectors(1:3, ind));
    if flag_is_rot
        fprintf(fid, '%f', push_actions.rot_angles(ind));
    elseif flag_is_trans
        fprintf(fid, '%f', push_actions.penetration_dist(ind));
    end
    fprintf(fid, ' %f\n', push_actions.moveclose_dist(ind));
    ct = ct + 1;
end
fclose(fid);
end

