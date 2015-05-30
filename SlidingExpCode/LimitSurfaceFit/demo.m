addpath(genpath('.'))
close all;
clear all;
rng(1);
% Two symetric bar demo:
%Pts = [-1,1;-1,1];
%PD = [5;5];
Np = 5;
r = 1;
m = 1; 
range = 8;
% Uniform random;
Pts = bsxfun(@minus, rand(2, Np),[0.5;0.5]) * r;

% Two uniform random;
% a = [ 0;5];
% b = [ 0;-5];
% num_pts1 = floor(Np/2);
% Pts(:, 1:1:num_pts1) = bsxfun(@plus, rand(2, num_pts1), a - [0.5;0.5]) * r;
% Pts(:, num_pts1+1:1:Np) = bsxfun(@plus, rand(2, Np - num_pts1), b - [0.5;0.5]) * r;

% On the rim
Angles = rand(Np, 1) * 2 * pi;
%Pts = r * [cos(Angles)';sin(Angles)'];

% Random weights.
PD = rand(Np, 1) * m;

% Same weights.
PD = ones(Np, 1) * m;

Nc = 200;
pho = r;
%%%%----------------%%%%
% COR style sampling...
CORs = GenerateRandomCORs(Pts, Nc);
[F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

% CORs = GenerateRandomCORs2(Nc, pho);
% [F, bv] = GenFVPairsFromPD(Pts, PD, CORs);


%%%%----------------%%%%%
%Unit body velocity style sampling.
% [V, bv] = GenBodyVelocities(Pts, PD, Nc);
% F = GetFrictionForce(V, Pts, PD);

% Erdman normalization.

F(3,:) = F(3,:) / pho;
bv(:,3) = bv(:,3) * pho;
bv = bsxfun(@rdivide, bv, sqrt(sum(bv.^2,2)));

figure;
axis tight;
view(-10, 20);
plot3(F(1,:), F(2,:), F(3,:), 'r*', 'Markersize', 6);
hold on;

%Convex Hull
%subplot(1,2,2);
figure;
axis tight;
k = convhull(F(1,:), F(2,:), F(3,:));
trisurf(k, F(1,:), F(2,:), F(3,:));
dir_F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));
numTrain = ceil(size(dir_F, 2) * 0.8);
dir_F_train = dir_F(:, 1:numTrain);
F_train = F(:,1:numTrain);
bv_train = bv(1:numTrain, :);
F_test = F(:, numTrain+1:end);
dir_F_test = dir_F(:, numTrain+1:end);
bv_test = bv(numTrain+1:end,:);

w_reg = 0;
w_vel = 1;
w_force = 1; 

[v, Q, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(F_train, bv_train', w_reg, w_vel, w_force);
pred_vel_train = GetVelFrom4thOrderPoly(v, dir_F_train);
pred_vel_test = GetVelFrom4thOrderPoly(v, dir_F_test);
err_v_test = pred_vel_test - bv_test;
disp('test velocity data error');
mean(sqrt(sum(err_v_test.^2,2)))
angles_test = acos(diag(bv_test * pred_vel_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)
 
w_force2 = 1;
w_reg2 = 0;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(F_train, bv_train', w_force2, w_reg2);
disp('Mean test error Linear');
[err, dev_angle] = EvaluateLinearPredictor(dir_F_test', bv_test, A)

%[r,fits] = FitPointsOnlyPolyOrder4(F', 1);

