% bv: N*3, before Erdman's normalization.
% Pts: 2*N;
% Pds: N*1.
% F: 3*N; 
% bv_double: after normalization.
function [F, bv_double] = ComputeTotalForceFromBodyVel(bv, Pts, Pds, pho)
nV = size(bv,1);
CORs = zeros(2, nV);

CORs(1,:) = -bsxfun(@rdivide, bv(:,2), bv(:,3));
CORs(2,:) = bsxfun(@rdivide, bv(:,1), bv(:,3));
%CORs = CORs * pho;
[V, bv_double] = GenVelocityDirections(Pts, CORs);
F = GetFrictionForce(V, Pts, Pds);
F(3,:) = F(3,:) / pho;
bv_double(:,3) = bv_double(:,3) * pho;
bv_double = bsxfun(@rdivide, bv_double, sqrt(sum(bv_double.^2,2)));
end

