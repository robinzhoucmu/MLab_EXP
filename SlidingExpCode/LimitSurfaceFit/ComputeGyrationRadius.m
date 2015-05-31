% Compute radius of gyration about the point of origin.
% Input:
% Pts: Np*2;
% Pds: Np*1.

function [pho] = ComputeGyrationRadius(Pts, Pds)
% Compute moment of inertia.
I = sum(sum(Pts.^2, 2).* Pds);
m = sum(Pds);
pho = sqrt(I/m);
end

