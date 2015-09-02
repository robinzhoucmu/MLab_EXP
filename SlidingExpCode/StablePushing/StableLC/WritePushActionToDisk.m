function [] = WritePushActionToDisk(push_actions, file_name, flag_stable)
fid = fopen(file_name, 'w');
num_pushes = size(push_actions.push_points,2);
fprintf(fid, '%d\n', num_pushes);
for ind = 1:1:num_pushes
    if (~flag_stable(ind))
        continue;
    end
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
end
fclose(fid);
end

