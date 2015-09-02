% Input:
% ContactInfo: containing push point (mid point of the fingers) and
% approach directions, in the frame with origin set as lower left right
% angle point.
% num_cor_per_push.
% pho, range_pho: control the COR sampling boundary. 
% rot_angle, moveclose_dist. 
% NOTES: Assume metric is in mm and degree.
function [PushActions] = GenerateRandomPushCORActionsGivenPushLocations(ContactInfo, num_cor_per_push, pho, range_pho, rot_angle, moveclose_dist)
num_contacts = length(ContactInfo);
ct = 1;
for ind_contact = 1:1:num_contacts
    % sample ratios -0.5 to 0.5 in the direction (vertical) of approach vector/edge normals.
    rx = rand([num_cor_per_push 1]) - 0.5;
    % sample ratios -1 to 1 in the direction of edges.
    ry = 2 * (rand([num_cor_per_push 1]) - 0.5);
    dir_x = ContactInfo.approach_vectors(:, ind_contact);
    dir_y = cross([0;0;1], dir_x);
    dir_x = dir_x / norm(dir_x);
    dir_y = dir_y / norm(dir_y);
    contact_pt = ContactInfo.push_points(:,ind_contact);
    for ind_cor = 1:1:num_cor_per_push
       PushActions.type{ct} = 'TwoPointRotation';
       PushActions.push_points(:,ct) = contact_pt;
       PushActions.approach_vectors(:,ct) = dir_x; 
       PushActions.cors(:,ct) = ...
           pho * range_pho * (dir_x * rx(ind_cor) + dir_y * ry(ind_cor)) + contact_pt;       
       if ry(ind_cor) > 0
        PushActions.rot_angles(ct) = rot_angle;
       else
        PushActions.rot_angles(ct) = -rot_angle;
       end
       PushActions.moveclose_dist(ct) = moveclose_dist;
       ct = ct+1;
    end
end
end

