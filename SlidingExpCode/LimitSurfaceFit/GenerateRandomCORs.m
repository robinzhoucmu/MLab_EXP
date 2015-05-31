% Input: 
% Pts_{2, Np}: x,y coordinates of pressure distribution points. 
% NC: number of samples to create random CORs.
% Output:
% CORs_{2, Nc}: x,y coordinates of randomly sampled CORs.
function [CORs] = GenerateRandomCORs(Pts, Nc, nFacetPts)
if (nargin == 2)
    nFacetPts = 50;
end

numP = size(Pts, 2);
CORs = zeros([2, Nc + nFacetPts * numP]);

% Compute point center.
pC = mean(Pts, 2);
% Compute average distance to center.
disp = bsxfun(@minus, Pts, pC);
avgR = mean(sqrt(sum(disp.^2)));
% Sample.
ratio_near = 4/10;
ratio_medium = 4/10;
num_near = Nc * ratio_near;
num_medium = Nc * ratio_medium;
CORs(:, 1:num_near) = bsxfun(@plus,(bsxfun(@minus, rand(2, num_near) , [0.5;0.5]))* 2 * avgR, pC);
CORs(:, num_near+1: num_near + num_medium) = ...
    bsxfun(@plus,(bsxfun(@minus, rand(2, num_medium) , [0.5;0.5]))* 20 * avgR, pC);
CORs(:, num_near+num_medium+1: Nc) = ...
    bsxfun(@minus, rand(2, Nc - num_near - num_medium), [0.5;0.5])* 100000;
% Rotation about support points.
CORs(:, Nc+1:Nc+ nFacetPts * numP) = repmat(Pts, [1 nFacetPts]);
end

