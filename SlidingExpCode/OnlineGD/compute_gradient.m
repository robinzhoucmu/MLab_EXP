% Input:
% a: The current coefficient.
% d: ith data point. 
% l: ith gradient form data point.
% v: ith velocity.
% E(3): Three matrix that map coefficients of the original 
% eta1: The control parameter for l2 regularization.
% eta2: The control parameter for velocity matching.
function [g] = compute_gradient(a, d, l, v, E, eta1, eta2)
n = size(a, 1);
% Compute estimated velocity.
est_v = zeros(3,1);
for i = 1:1:3
    est_v(i) = l' * (E{i} * a);
end
% Compute velocity alignment scalar.
s = est_v' * v;
% Compute ds/da.
ds_da = zeros(n,1);
for i = 1:1:3
    ds_da = ds_da + v(i) * E{i}' * l;
end 
% Compute gradient.
g1 = (d'*a - 1) * d; 
g2 = eta1 * a;
g3 = zeros(n,1);
for i = 1:1:3
    g3 = g3 + eta2 * (E{i}' * l - v(i) * ds_da) * (l' * E{i} * a - s * v(i));
end
% norm(g1)
% norm(g2)
% norm(g3)
g = g1 + g2 + g3;
end

