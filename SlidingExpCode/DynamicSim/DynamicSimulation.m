% Represent planar pose as 3*3 homogeneous matrix. [Rot,trans;0,0,1];
% Input: 
% m: mass.
% pho: radius of gyration. 
% coeffs for poly4 or quadratic.
% V0: 3*1;initial velocity in local object frame.
% T0: 3*3;initial object pose in global frame.
% mode: specify the limit surface fitting mode.
% Output:
% pose in global frame
function [T_record, pos_record] = DynamicSimulation(m, pho, coeffs, V0, Pose0, dt, mode)
cur_V = V0;
cur_Pose = Pose0;
eps_V = 1e-3;
ind = 1;
t0 = dt;
elapsed_time = 0;
while (norm(cur_V) > eps_V)
    dt = t0 / sqrt(ind);
    elapsed_time = elapsed_time + dt;
    T_record{ind} = cur_Pose;
    pos_record(ind,1:2) = cur_Pose(1:2,3)';
    pos_record(ind, 3) = acos(cur_Pose(1,1)) * 180 / pi;
    cur_V2 = cur_V;
    cur_V2(3) = cur_V2(3) * pho; 
    F = FindForceGivenVel(cur_V2, coeffs, mode);
    Acc = - F / m;
    % Change the angular component back to angular acceleration (in radian).
    Acc(3) = Acc(3) / pho
    % Compute next velocity.
    nxt_V = cur_V + Acc * dt;
    % Compute displacement in local frame.
    D = 0.5 * (nxt_V + cur_V) * dt;
    % Represent displacement as homog matrix. 
    dx = D(1)
    dy = D(2)
    dtheta = D(3)
    homog_D = [cos(dtheta), -sin(dtheta), dx;
               sin(dtheta), cos(dtheta), dy;
               0, 0, 1];
    % Compute next object pose in global frame.
    nxt_Pose = cur_Pose * homog_D;

    % Update all current status
    cur_Pose = nxt_Pose
    cur_V = nxt_V
    ind = ind + 1;
    %if (ind > 10) break;
end
elapsed_time
end

