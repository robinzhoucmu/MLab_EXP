% Comparisons among several techniques (poly4, poly2, gp, rf).
function [ err_angles_train, err_angles_test, info] = MultiEvaluatation(num_evals, num_pts, num_cors, facet_pts, r_train, noise, options_pt, options_pd)
num_methods = 4;
err_angles_test = zeros(num_evals, num_methods);
err_angles_train = zeros(num_evals, num_methods);

for i = 1:1:num_evals
    [Pts] = SampleSupportPoint(num_pts, options_pt);
    size(Pts)
    %Pts = GridSupportPoint(num_pts, options_pt);
    [Pds] = AssignPressure(Pts, options_pd);
    pho = ComputeGyrationRadius(Pts, Pds);

    Nc = num_cors;
    % COR style sampling...
    CORs = GenerateRandomCORs(Pts', Nc, facet_pts);
    [F, bv] = GenFVPairsFromPD(Pts', Pds, CORs);

    % CORs = GenerateRandomCORs2(Nc, pho);
    % [F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

    % Erdman normalization.
    F(3,:) = F(3,:) / pho;
    bv(:,3) = bv(:,3) * pho;
    bv = bsxfun(@rdivide, bv, sqrt(sum(bv.^2,2)));
    
    % Split training and testing. 
    split_train = r_train;
    numTrain = ceil(size(F, 2) * split_train)
    % Add noise to training force and vel.
    eps_noise_f = noise.f;
    F(:,1:numTrain) = F(:,1:numTrain) + eps_noise_f * randn(3, numTrain);
    eps_noise_v = noise.v;
    bv(1:numTrain,:) = bv(1:numTrain,:) + eps_noise_v * randn(numTrain, 3);
    
    dir_F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));
    info.numTrain = numTrain;
    info.totData = size(dir_F, 2);
    dir_F_train = dir_F(:, 1:numTrain);
    F_train = F(:,1:numTrain);
    bv_train = bv(1:numTrain, :);
    F_test = F(:, numTrain+1:end);
    dir_F_test = dir_F(:, numTrain+1:end);
    bv_test = bv(numTrain+1:end,:);
    
    dir_F_train = F_train; 
    % 4th order poly with convexity constraint. 
    ind_method_poly4_cvx = 1;
    w_reg = 0;
    w_vel = 1;
    w_force = 1; 
    flag_convex = 1;
    [coef_poly4_cvx, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(dir_F_train, bv_train', w_reg, w_vel, w_force, flag_convex);
    [err_test, dev_angle_test] = EvaluatePoly4Predictor(dir_F_test', bv_test, coef_poly4_cvx);
    err_angles_test(i, ind_method_poly4_cvx) = dev_angle_test;
    [err_train, dev_angle_train] = EvaluatePoly4Predictor(dir_F_train', bv_train, coef_poly4_cvx);
    err_angles_train(i, ind_method_poly4_cvx) = dev_angle_train;

    % 4th order poly w/o convexity constraint.
    ind_method_poly4 = 2;
%     w_reg = 0;
%     w_vel = 1;
%     w_force = 0; 
    flag_convex = 0;
    [coef_poly4, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(dir_F_train, bv_train', w_reg, w_vel, w_force, flag_convex);
    [err_test, dev_angle_test] = EvaluatePoly4Predictor(dir_F_test', bv_test, coef_poly4);
    err_angles_test(i, ind_method_poly4) = dev_angle_test;
    [err_train, dev_angle_train] = EvaluatePoly4Predictor(dir_F_train', bv_train, coef_poly4);
    err_angles_train(i, ind_method_poly4) = dev_angle_train;
    
%     % Projection onto SOS cone. 
%     [sosE, sosA, sosB] = get_poly4_parameters();
%     [coef_poly4_proj] = projection_sos_convex_sdp(coef_poly4, sosA, sosB);
%     [err_test, dev_angle_test] = EvaluatePoly4Predictor(dir_F_test', bv_test, coef_poly4_proj)
% 
%     coef_poly4_cvx
%     coef_poly4
%     coef_poly4_proj
%     norm(coef_poly4_proj - coef_poly4)
    
    
    % 2nd order poly. 
    ind_method_poly2 = 3;
    w_force2 = 0;
    w_reg2 = 1;
    [A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(dir_F_train, bv_train', w_force2, w_reg2);
    [err_test, dev_angle_test] = EvaluateLinearPredictor(dir_F_test', bv_test, A);
    err_angles_test(i, ind_method_poly2) = dev_angle_test;
    [err_train, dev_angle_train] = EvaluateLinearPredictor(dir_F_train', bv_train, A);
    err_angles_train(i, ind_method_poly2) = dev_angle_train;

    

%     % random forest.
%     ind_method_rf = 3;
%     options_rf.ntrees = 50;
%     options_rf.leaf_size = 5;
%     [trees, angles_train, angles_test] = RF_Fitting(options_rf, dir_F_train', bv_train, dir_F_test', bv_test);
%     err_angles_test(i, ind_method_rf) = angles_test;
%     err_angles_train(i, ind_method_rf) = angles_train;
    
    % gp with poly kernel.
    ind_method_gp = 4;
    [hyp, err_angle_train, err_angle_test] = GP_Fitting(dir_F_train', bv_train, dir_F_test', bv_test);
    err_angles_test(i, ind_method_gp) = err_angle_test;
    err_angles_train(i, ind_method_gp) = err_angle_train;
end

end

