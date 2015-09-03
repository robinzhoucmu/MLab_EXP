% Input:
% finger_width (in mm)
% H_tf: com frame w.r.t. lower left corner.
function [pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale)
if (nargin < 4)
    unit_scale = 1000;
end
num_pushes = size(push_actions.push_points, 2);
R_z = [cos(pi/2), -sin(pi/2), 0;
       sin(pi/2), cos(pi/2), 0;
       0, 0, 1];
 pt_contacts = cell(num_pushes, 1);
 pt_outward_normals = cell(num_pushes, 1);
for i = 1:1:num_pushes
    d = R_z * push_actions.approach_vectors(:,i) * fingers_width / 2;
    fingers_pos = [push_actions.push_points(:,i) - d, push_actions.push_points(:,i) + d];
    fingers_pos_append1 = H_tf \ [fingers_pos;1,1];
    fingers_pos = fingers_pos_append1(1:3,:);
    %fingers_pos = bsxfun(@minus, fingers_pos, trans);
    pt_contacts{i} = fingers_pos(1:2,:) / unit_scale;
    pt_outward_normals{i} = repmat(-push_actions.approach_vectors(1:2, i), [1,2]);
end
end

