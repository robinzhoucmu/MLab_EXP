% Generate random force velocity pairs from a given pressure distribution
% Input: 
% Pts_{2,Np}: (x,y) coordinates of the pressure distribution points.
% PD: support force at the pressure distribution points.
% Nc: num of random CORs to sample in the x-y plane for generating F-V pairs. 
% Output:
% F_{3, Nc}: The forces and moment (about the COM) for each CORs. 
% V_: velocity matrix, where each 2 rows represent the velocities
function [F, bv] = GenFVPairsFromPD(Pts, PD, CORs)

[V, bv] = GenVelocityDirections(Pts, CORs);
[F] = GetFrictionForce(V, Pts, PD);
end

