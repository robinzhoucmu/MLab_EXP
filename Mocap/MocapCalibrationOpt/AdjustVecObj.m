% This function adjusts the object vector (manual measurement initialization)
% represented in the robot tool frame
% Input:
% P: 3*N, Set of points(tool center) in robot base frame.
% Q: 3*N, Corresponding set of mocap points transformed back to the base frame
% Output:
% v: The refitted vector that corresponds to the marker point shifting
% vector in base frame. We are assuming the tool phalange is always pointing downwards.
% Hence, the rotation matrix that represents tool frame in base frame is fixed. 
function [v] = AdjustVecObj(P, Q)
v = mean(Q - P, 2);
end

