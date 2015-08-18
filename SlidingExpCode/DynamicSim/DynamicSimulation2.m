% Compared with DynamicSimulation:
% V0 and Pose0 are all in global frame.
function [T_record, pos_record, Time] = DynamicSimulation2(m, pho, coeffs, V0, Pose0, dt, mode)
cur_V = V0;
cur_Pose = Pose0;
eps_V = 1e-3;
ind = 1;
t0 = dt;
elapsed_time = 0; 
Time(1) = 0;
while (norm(cur_V) > eps_V)
    dt = t0 / sqrt(ind);
    elapsed_time = elapsed_time + dt;
    T_record{ind} = cur_Pose;
    pos_record(ind,1:2) = cur_Pose(1:2,3)';
    pos_record(ind, 3) = acos(cur_Pose(1,1));
    cur_V2 = cur_V;
    R = cur_Pose(1:2,1:2);
    % Represent in local frame.
    cur_V2(1:2) = R' * cur_V(1:2);
    cur_V2(3) = cur_V2(3) * pho; 
    F = FindForceGivenVel(cur_V2, coeffs, mode);
    Acc = -F / m;
    % Change the angular component back to angular acceleration (in radian).
    Acc(3) = Acc(3) / pho;
    % Change first 2 components to global frame. 
    Acc(1:2) = R * Acc(1:2);
    % Compute next velocity.
    nxt_V = cur_V + Acc * dt;
    % Compute displacement in local frame.
    %D = 0.5 * (nxt_V + cur_V) * dt;
    D = cur_V * dt;
    % Represent displacement as homog matrix. 
    dx = D(1);
    dy = D(2);
    dtheta = D(3);
    nxt_Pose = eye(3,3);
    theta = pos_record(ind,3);
    nxt_Pose(1:2,1:2) = [cos(theta + dtheta), -sin(theta + dtheta); sin(theta+dtheta), cos(theta+dtheta)];
    nxt_Pose(1:2,3) = cur_Pose(1:2,3) + [dx;dy];
    % Update all current status
    cur_Pose = nxt_Pose;
    cur_V = nxt_V;
    ind = ind + 1;
    Time(ind) = Time(ind-1) + dt;
end
elapsed_time
end
