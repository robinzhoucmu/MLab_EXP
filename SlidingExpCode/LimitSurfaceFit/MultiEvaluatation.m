% Comparisons among several techniques (poly4, poly2, gp, rf).
function [ err_angles_train, err_angles_test ] = MultiEvaluatation(num_evals, num_pts, num_cors, r_train, options_pt, options_pd)
num_methods = 4;
err_angles_test = zeros(num_evals, num_methods);
err_angles_train = zeros(num_evals, num_methods);

for i = 1:1:num_evals
    [Pts] = SampleSupportPoint(num_pts, options_pt);
    [Pds] = AssignPressure(Pts, options_pd);
    pho = ComputeGyrationRadius(Pts, Pds);

    Nc = num_cors;
    % COR style sampling...
    CORs = GenerateRandomCORs(Pts', Nc, 2);
    [F, bv] = GenFVPairsFromPD(Pts', Pds, CORs);

    % CORs = GenerateRandomCORs2(Nc, pho);
    % [F, bv] = GenFVPairsFromPD(Pts, PD, CORs);

    % Erdman normalization.
    F(3,:) = F(3,:) / pho;
    bv(:,3) = bv(:,3) * pho;
    bv = bsxfun(@rdivide, bv, sqrt(sum(bv.^2,2)));
    
    % Split training and testing. 
    dir_F = bsxfun(@rdivide, F, sqrt(sum(F.^2)));
    split_train = r_train;
    numTrain = ceil(size(dir_F, 2) * split_train);
    dir_F_train = dir_F(:, 1:numTrain);
    F_train = F(:,1:numTrain);
    bv_train = bv(1:numTrain, :);
    F_test = F(:, numTrain+1:end);
    dir_F_test = dir_F(:, numTrain+1:end);
    bv_test = bv(numTrain+1:end,:);
    
    % 4th order poly. 
    ind_method_poly4 = 1;
    w_reg = 0;
    w_vel = 1;
    w_force = 0; 
    [coef_poly4, Q, xi, delta, pred_v, s] = Fit4thOrderPolyCVX(dir_F_train, bv_train', w_reg, w_vel, w_force);
    [err_test, dev_angle_test] = EvaluatePoly4Predictor(dir_F_test', bv_test, coef_poly4);
    err_angles_test(i, ind_method_poly4) = dev_angle_test;
    [err_train, dev_angle_train] = EvaluatePoly4Predictor(dir_F_train, bv_train, coef_poly4);
    err_angles_train(i, ind_method_poly4) = dev_angle_train;
   
    % 2nd order poly. 
    ind_method_poly2 = 2;
    w_force2 = 0;
    w_reg2 = 0;
    [A, xi_elip, delta_elip, pred_v_lr_train, s_lr] = FitElipsoidForceVelocityCVX(dir_F_train, bv_train', w_force2, w_reg2);
    [err_test, dev_angle_test] = EvaluateLinearPredictor(dir_F_test', bv_test, A)
    err_angles_test(i, ind_method_poly2) = dev_angle_test;
    [err_train, dev_angle_train] = EvaluateLinearPredictor(dir_F_train', bv_train, A)
    err_angles_train(i, ind_method_poly2) = dev_angle_train;


    % random forest.
    ind_method_rf = 3;
    options_rf.ntrees = 50;
    options_rf.leaf_size = 5;
    [trees, angles_train, angles_test] = RF_Fitting(options, dir_F_train', bv_train, dir_F_test', bv_test);
    err_angles_test(i, ind_method_rf) = angles_test;
    err_angles_train(i, ind_method_tf) = angles_train;
    
    % gp with poly kernel.
    ind_method_gp = 4;
    [hyp, err_angle_train, err_angle_test] = GP_Fitting(dir_F_train', bv_train, dir_F_test', bv_test)
    err_angles_test(i, ind_method_gp) = err_angle_test;
    err_angles_train(i, ind_method_gp) = err_angle_train;
end

end

