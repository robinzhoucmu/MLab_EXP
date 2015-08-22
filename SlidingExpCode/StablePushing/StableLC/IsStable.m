% check whether pushing will be stable or not given limit surface.
% Input:
% vel: velocity assume with angular velocity normalized. 
% pt_contacts: [x1,x2;y1,y2]; 
% pt_outward_normals: [nx1,nx2;ny1,ny2];
% mu: coefficient of friction at point of contacts, assume same for both contact points.
% pho: charateristic length, e.g., radius of gyration.
% lc_coeffs: coefficients for given limit surface.
% lc_type: 'quadratic' or 'poly4'.
function [resnorm, x] = IsStable(vel, pt_contacts, pt_outward_normals, mu, lc_coeffs, pho, lc_type)
if nargin < 6 
    if size(lc_coeffs, 1) == 3 && size(lc_coeffs,2) == 3
        lc_type = 'quadratic';
    else
        lc_type = 'poly4';
    end
end
% Compute Edges of friction cone. 
[fc_edges] = ComputeFrictionConeEdges(pt_contacts, pt_outward_normals, mu, pho);
% Compute Edges of velocity cone.
[vc_edges] = ComputeVelConeGivenFC(fc_edges, lc_coeffs, lc_type);

% Check if desired velocity is in the motion cone.
[x,resnorm] = lsqnonneg(vc_edges,  vel);
end

