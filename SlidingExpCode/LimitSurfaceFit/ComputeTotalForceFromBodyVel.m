% bv: N*3, before Erdman's normalization.
% Pts: 2*N;
% Pds: N*1.
% F: N*3; Friction Force object applied on the surface.  
% bv_double: after normalization.
function [F] = ComputeTotalForceFromBodyVel(bv, Pts, Pds)
nV = size(bv,1);
nP = size(Pts,2);
F = zeros(nV,3);
for i = 1:1:nV
    for j = 1:1:nP
        vx = -Pts(2,j) * bv(i,3) + bv(i,1);
        vy = Pts(1,j) * bv(i,3) + bv(i,2);
        dir_v = [vx,vy]/sqrt(vx^2+vy^2);
        fx = Pds(j) * dir_v(1);
        fy = Pds(j) * dir_v(2);
        tau = Pts(1,j) * fy - Pts(2,j) * fx;
        F(i,:) = F(i,:) + [fx,fy,tau];
    end
end
% CORs = zeros(2, nV);
% 
% CORs(1,:) = -bsxfun(@rdivide, bv(:,2), bv(:,3));
% CORs(2,:) = bsxfun(@rdivide, bv(:,1), bv(:,3));
% %CORs = CORs * pho;
% [V, bv_double] = GenVelocityDirections(Pts, CORs);
% F = GetFrictionForce(V, Pts, Pds);
% F(3,:) = F(3,:) / pho;
% bv_double(:,3) = bv_double(:,3) * pho;
% bv_double = bsxfun(@rdivide, bv_double, sqrt(sum(bv_double.^2,2)));
end

