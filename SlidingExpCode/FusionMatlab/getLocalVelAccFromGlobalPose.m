% Compute velocity w.r.t to local object frame.
% Input: trajectory of 2d pos, N*3, (x,y,theta(degrees)) w.r.t global robot base frame.
% Output: velocities measured w.r.t to moving object local frame.
function [Vel,Acc] = getLocalVelAccFromGlobalPose(pos_2d, t)
diff_t = t(2:end) - t(1:end-1);
ind_filter_t = diff_t == 0;
t(ind_filter_t) = [];
pos_2d(ind_filter_t,:) = [];

 N = size(pos_2d, 1);
% Vel = zeros(N-2, 3);
% Acc = zeros(N-2, 3);

v_r = bsxfun(@rdivide, pos_2d(3:end,:) - pos_2d(2:end-1,:), t(3:end) - t(2:end-1));
v_l = bsxfun(@rdivide, pos_2d(2:end-1,:) - pos_2d(1:end-2,:), t(2:end-1) - t(1:end-2));
Vel = (v_l + v_r) / 2;
Acc = bsxfun(@rdivide, v_r - v_l, t(3:end) - t(1:end-2));

for i = 2:1:N-1
    theta = pos_2d(i,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    Vel(i-1,1:2) = (R' * Vel(i-1,1:2)')';
    Acc(i-1,1:2) = (R' * Vel(i-1,1:2)')';
end
% Vel(:,1:2) = Vel(:,1:2) * 1000;
% Acc(:, 3) = Acc(:,3) * 1000000;
% Acc(:,1:2) = Acc(:,1:2) * 1000;
%standardize(Vel);
%standardize(Acc);
end

