% pos_2d: 6*N. 1-3 is pos, 4-6 is vel, in the global frame. 
% Vel, Force: 3*N. third component normalized.
function [Vel, Force ] = computeLocalVelAndForce( pos_2d, pho, coeffs, mode)
num_poses = size(pos_2d, 2);
Vel = zeros(3, num_poses);
Force = zeros(3, num_poses);
for i = 1:1:num_poses
theta = pos_2d(3,i);
R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
v = pos_2d(4:6, i);
% Change of frame for translational velocity.
v(1:2) = R' * v(1:2);
v(3) = v(3) * pho;
f = FindForceGivenVel(v, coeffs, mode);
Vel(:, i) =  v;
Force(:, i) = f;
end
end

