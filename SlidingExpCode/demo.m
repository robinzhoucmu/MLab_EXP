addpath(genpath('.'))
close all;
clear all;
rng(1);
% Two symetric bar demo:
%Pts = [-1,1;-1,1];
%PD = [5;5];
Np = 3;
r = 10;
m = 2; 
range = 8;
% Uniform random;
%Pts = bsxfun(@minus, rand(2, Np),[0.5;0.5]) * r;

% Two uniform random;
% a = [ 0;5];
% b = [ 0;-5];
% num_pts1 = floor(Np/2);
% Pts(:, 1:1:num_pts1) = bsxfun(@plus, rand(2, num_pts1), a - [0.5;0.5]) * r;
% Pts(:, num_pts1+1:1:Np) = bsxfun(@plus, rand(2, Np - num_pts1), b - [0.5;0.5]) * r;

% On the rim
Angles = rand(Np, 1) * 2 * pi;
Pts = r * [cos(Angles)';sin(Angles)'];

% Random weights.
PD = rand(Np, 1) * m;

% Same weights.
%PD = ones(Np, 1) * m;

Nc = 20;

%%%%----------------%%%%
% COR style sampling...
CORs = GenerateRandomCORs(Pts, Nc);
[F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

F = F(:,1:end/2);
bv = bv(1:end/2,:);

%F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));


%%%%----------------%%%%%
% Unit body velocity style sampling.
% [V, bv] = GenBodyVelocities(Pts, PD, Nc);
% F = GetFrictionForce(V, Pts, PD);


%axis equal
%subplot(1,2,1);
figure;
axis tight;
view(-10, 20);
plot3(F(1,:), F(2,:), F(3,:), 'r*', 'Markersize', 6);
hold on;
%figure;
%scatter(Pts(1,:), Pts(2,:));
%figure;
%scatter(CORs(1,:), CORs(2,:));
% xData = zeros(size(F,2)*2, 3);
% yData = zeros(size(F,2)*2, 1);
% xData(1:1:size(F,2),:) = F'*1.1;
% xData(size(F,2)+1:1:2*size(F,2),:) = F'*0.9;
% yData(1:1:size(F,2),:) = 1;
% yData(size(F,2)+1:1:2*size(F,2),:) = -1;

%Convex Hull
%subplot(1,2,2);
figure;
axis tight;
k = convhull(F(1,:), F(2,:), F(3,:));
trisurf(k, F(1,:), F(2,:), F(3,:));
dir_F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));
numTrain = ceil(size(dir_F, 2) * 0.8);
dir_F_train = dir_F(:, 1:numTrain);
bv_train = bv(1:numTrain, :);
dir_F_test = dir_F(:, numTrain+1:end);
bv_test = bv(numTrain+1:end,:);

[v, Q, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(dir_F_train, bv_train', 0, 100, 1);
pred_vel_train = GetVelFrom4thOrderPoly(v, dir_F_train);
pred_vel_test = GetVelFrom4thOrderPoly(v, dir_F_test);
err_v_test = pred_vel_test - bv_test;
disp('test velocity data error');
mean(sqrt(sum(err_v_test.^2,2)))
% % Least square quadratic fitting.
% figure;
% %subplot(2,2,3);
% plot3(F(1,:), F(2,:), F(3,:), 'r.');
% [ center, radii, evecs, v, fits_ellipsoid ] = ellipsoid_fit( [F'; -F']);
% maxd = max(abs(F), [], 2) * range;
% step = maxd / 100;
% [ x, y, z ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);
% 
% Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
%           2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
%           2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
% p = patch( isosurface( x, y, z, Ellipsoid, 1 ) );
% set( p, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none' );
% view(-10, 20);
% axis vis3d;
% 
lambda = 0; gamma = 1000;
[A, xi_elip, delta_elip] = FitElipsoidForceVelocityCVX(dir_F, bv', lambda, gamma);
% 
% figure;
% %subplot(2,2,4);
% plot3(F(1,:), F(2,:), F(3,:), 'r.');
% 
% maxd = max(abs(F), [], 2) * range;
% step = maxd / 100;
% [ x, y, z ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);
% 
% Ellipsoid = A(1,1) * x .* x +   A(2,2) * y.* y + A(3,3) * z .* z + ...
%           2 * A(1,2) * x .* y + 2 * A(1,3) * x .* z + 2 * A(2,3) * y .* z;
% p = patch( isosurface( x, y, z, Ellipsoid, 1 ) );
% set( p, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none' );
% view(-10, 20);
% camlight;
% lighting phong;
% axis tight;

%[r,fits] = FitPointsOnlyPolyOrder4(F', 1);

