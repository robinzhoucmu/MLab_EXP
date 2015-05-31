addpath(genpath('.'))
close all;
clear all;
rng(1);
Np = 6;
r = 2;
m = 1; 
range = 10;

% On the rim
Angles = rand(Np, 1) * 2 * pi;
Pts = r * [cos(Angles)';sin(Angles)'];


% Uniform random;
%Pts = bsxfun(@minus, rand(2, Np),[0.5;0.5]) * r;


% Random weights.
PD = rand(Np, 1) * m;

% Same weights.
PD = ones(Np, 1) * m;

Nc = 20;

%%%%----------------%%%%
% COR style sampling...
CORs = GenerateRandomCORs(Pts, Nc);
[F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

% Normalize F for each dimension.
F = bsxfun(@times, F, 1.0 ./ sqrt(mean(F.^2, 2)));
%Randomly shuffle F and bv;
ind_rand = randperm(size(F,2));
F = F(:, ind_rand);
bv = bv(ind_rand, :);

figure;
axis tight;
view(-10, 20);
plot3(F(1,:), F(2,:), F(3,:), 'r*', 'Markersize', 6);

figure;
axis tight;
k = convhull(F(1,:), F(2,:), F(3,:));
trisurf(k, F(1,:), F(2,:), F(3,:));

% Use the first half to compute an initializer. 
r0 = 0.1;
eta1 = 0;
eta2 = 0;
passes = 10;
lr_rate =  0.5;
pre_train_num = floor(size(F,2) * r0);
if (pre_train_num > 0)
    [a0, Q0, xi0, delta0] = Fit4thOrderPolyCVX(F(:,1:pre_train_num), bv(1:pre_train_num,:)', eta1, eta2);
    [a, xi_a] = online_poly4(F(:,pre_train_num+1:end), bv(pre_train_num+1:end,:)', eta1, eta2, passes, lr_rate, a0, Q0);
else
[a, xi_a] = online_poly4(F, bv', eta1, eta2, passes, lr_rate);
end
% Batch SDP.
[v, Q, xi_v, delta] = Fit4thOrderPolyCVX(F, bv', eta1, eta2);
%a0
a
sum(xi_a)
v
sum(xi_v)
