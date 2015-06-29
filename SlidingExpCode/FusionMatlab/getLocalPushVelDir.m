% Input: obj_pos_2d(N*3), robot_finger(N*2) in global frame.
% Output: Pushing velocity and point of contact in the local frame.  
function [VelPush, PtContact] = getLocalPushVelDir(obj_pos_2d, robot_finger, t)

diff_t = t(2:end) - t(1:end-1);
ind_filter_t = diff_t == 0;
t(ind_filter_t) = [];
obj_pos_2d(ind_filter_t,:) = [];
robot_finger(ind_filter_t,:) = [];

N = size(obj_pos_2d,1);
VelPush = zeros(N-1, 2);
PtContact = zeros(N-1, 2);

pusher_disp = bsxfun(@minus, robot_finger(2:end,:), robot_finger(1:end-1, :));
pusher_pt = bsxfun(@minus, robot_finger, obj_pos_2d(:,1:2));

for i = 1:N-1
    theta = obj_pos_2d(i,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    VelPush(i,:) = (R' * pusher_disp(i,:)')' / (t(i+1) - t(i));
    PtContact(i,:) = (R' * pusher_pt(i,:)')';
end
% Velocity magnitude filter.
% VelPushNorm = sqrt(sum(VelPush.^2,2));
% eps_moving = 1e-3;
% ind_still = VelPushNorm < 1e-3;
% VelPush(ind_still, :) = [];
% PtContact(ind_still, :) = [];

end

