% Input: 
% object 2d pose (N*3), forces (fx, fy) (N*2) and robot x,y (N*2) in global frame
% Output: Wrenches in object local frame.
function [Wrenches] = ComputeWrench(Pos_2d, Forces, Robot_xy)
N = size(Pos_2d, 1);
Wrenches = zeros(N, 3);
for i = 1:1:N
    theta = Pos_2d(i,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    Wrenches(i,1:2) = (R' * Forces(i,1:2)')';
    r_pc = Robot_xy(i,1:2) - Pos_2d(i,1:2);
    r_pc = (R' * r_pc')';
    fx = Wrenches(i,1);
    fy = Wrenches(i,2);
    pc_x = r_pc(1);
    pc_y = r_pc(2);
    torque = pc_x * fy - pc_y * fx;
    Wrenches(i,3) = torque;
end
end

