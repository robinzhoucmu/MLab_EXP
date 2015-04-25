% Input:
% Pts_{2, Np}: x,y coordinates of the pressure distribution points;
% PD_{Np, 1}: weights for the pressure distribution points;
% Nv: the number of velocity vector to sample (approximate).

% Output:
% V_{2*Nv, Np}: the velocity matrix for each point, 2*Nv is approximate. 
% v_{Nv, 3}: the body velocity matrix.
function [V, v] = GenBodyVelocities(Pts, PD, Nv)
Np = size(Pts, 2);
% Rejection sample Nv number of velocities.
Ns = ceil(Nv * (8 / (4*pi/3))); 
% Sample from [-1,1] cube.
t = bsxfun(@minus, rand(Ns, 3), [0.5,0.5,0.5]) * 2;
% Rejection sampling.
ind = sum(t.^2, 2) <= 1;
v = t(ind,:);
num_v = size(v,1);
% Project back to the sphere.
v = bsxfun(@rdivide, v, sqrt(sum(v.^2, 2)));


% Compute COM.
COM = Pts * PD / sum(PD);

% Compute velocity.
Disp = bsxfun(@minus, Pts, COM);
V = zeros(num_v * 2, Np);
% The vx part for rotation about COM.
V(1:2:2*num_v, :) = bsxfun(@times, repmat(Disp(2,:), [num_v, 1]), v(:,3));
% Add the tranlational vx part.
V(1:2:2*num_v, :) = bsxfun(@plus, V(1:2:2*num_v, :), v(:,1));

% The vy part for rotation about COM.
V(2:2:2*num_v, :) = bsxfun(@times, repmat(Disp(1,:), [num_v, 1]), v(:,3));
% Add the translational vy part.
V(2:2:2*num_v, :) = bsxfun(@plus, V(2:2:2*num_v, :), v(:,2));

% Compute the velocity norm. Add eps.
Nv = 1./sqrt(V(1:2:2*num_v,:).^2 + V(2:2:2*num_v,:).^2 + eps);

% Normalize the velocities.
V(1:2:2*num_v,:) = bsxfun(@times, V(1:2:2*num_v,:), Nv);
V(2:2:2*num_v,:) = bsxfun(@times, V(2:2:2*num_v,:), Nv);

end

