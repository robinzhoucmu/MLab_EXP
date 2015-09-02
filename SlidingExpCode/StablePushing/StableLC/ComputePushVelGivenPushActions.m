% Input: push_actions with information w.r.t to local frame (point of
% origin at lower left corner).
% Output: push_vels 3*N with angular part normalized. point of origin
% change to COM.
function [push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale)
num_pushes = size(push_actions.push_points, 2);
push_vels = zeros(3, num_pushes);

for i = 1:1:num_pushes
    if strcmp(push_actions.type{i},'TwoPointRotation')
        % change COR frame.
        cor_append1 = H_tf \ [push_actions.cors(1:3, i);1];
        cor = cor_append1(1:3);
        cor = cor / unit_scale;
        z = sqrt(cor(1)^2 + cor(2)^2 + 1);
        % CCW.
        if (push_actions.rot_angles(i) > 0)
            push_vels(1, i) =  cor(2) / z;
            push_vels(2, i) = - cor(1) / z;
            push_vels(3, i) = 1 / z;
        else
        % CW.
            push_vels(1, i) =  - cor(2) / z;
            push_vels(2, i) = cor(1) / z;
            push_vels(3, i) = - 1 / z;
        end
    end
end
% normalize angular component.
push_vels(3,:) = push_vels(3,:) * pho;
push_vels = bsxfun(@rdivide, push_vels, sqrt(sum(push_vels.^2)));

end

