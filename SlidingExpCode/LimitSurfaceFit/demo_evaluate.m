close all;
clear all;
rng(1);

% Sample points randomly on a polygon, e.g., triangle.
%options_pt.mode = 'polygon';
%options_pt.vertices = [0,0; 0,1; 1,0];
options_pt.mode = 'circle';
options_pt.range = 4;
numPts = 5;
[Pts] = SampleSupportPoint(numPts, options_pt);

%Pts = options_pt.vertices;

% Assign pressure weights for the support points.
options_pd.mode = 'uniform';
options_pd.coef = [1 1 1];
[Pds] = AssignPressure(Pts, options_pd);
pho = ComputeGyrationRadius(Pts, Pds);
%pho = 1;

Nc = 100;
% COR style sampling...
CORs = GenerateRandomCORs(Pts', Nc, 30);
[F, bv] = GenFVPairsFromPD(Pts', Pds, CORs);

% CORs = GenerateRandomCORs2(Nc, pho);
% [F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

% Erdman normalization.

F(3,:) = F(3,:) / pho;
bv(:,3) = bv(:,3) * pho;
bv = bsxfun(@rdivide, bv, sqrt(sum(bv.^2,2)));

h = VisualizeForceVelPairs(F, bv');

figure;
axis tight;
view(-10, 20);
plot3(F(1,:), F(2,:), F(3,:), 'r*', 'Markersize', 6);
hold on;

%---------------------------------------%

figure;
axis tight;
k = convhull(F(1,:), F(2,:), F(3,:));
trisurf(k, F(1,:), F(2,:), F(3,:));
dir_F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));

split_train = 0.1;
numTrain = ceil(size(dir_F, 2) * split_train);
dir_F_train = dir_F(:, 1:numTrain);
F_train = F(:,1:numTrain);
bv_train = bv(1:numTrain, :);
F_test = F(:, numTrain+1:end);
dir_F_test = dir_F(:, numTrain+1:end);
bv_test = bv(numTrain+1:end,:);

w_reg = 0;
w_vel = 1;
w_force = 0.5; 

[v, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(F_train, bv_train', w_reg, w_vel, w_force);
pred_vel_test = GetVelFrom4thOrderPoly(v, dir_F_test);
err_v_test = pred_vel_test - bv_test;
disp('Test velocity data error');
mean(sqrt(sum(err_v_test.^2,2)))
angles_test = acos(diag(bv_test * pred_vel_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)

[err_test, dev_angle_test] = EvaluatePoly4Predictor(dir_F_test', bv_test, v);
dev_angle_test

 
w_force2 = 0.5;
w_reg2 = 0;
[A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(F_train, bv_train', w_force2, w_reg2);
disp('Mean test error Linear');
[err, dev_angle] = EvaluateLinearPredictor(dir_F_test', bv_test, A)

w_force3 = 0;
w_reg3 = 0;
[A2, a2, err_angle] = FitElipsoidSdpCon(F_train, bv_train', w_force3, w_reg3);
disp('Mean test error Linear');
[err2, dev_angle2] = EvaluateLinearPredictor(dir_F_test', bv_test, A2)
