function [exp_record] = MultiEval2(num_evals, num_pts, num_cors, options_pt, options_pd, r_facet, r_train, noise)
if (nargin < 8)
    noise.f = 0;
    noise.v = 0;
end
exp_record = {};
for ind_eval = 1:1:num_evals
    test_data = {};
    train_data = {};
    validation_data = {};
    fprintf('-----------------------\n');
    fprintf('percentage completed:%f\n', (ind_eval-1) * 100 / num_evals);
    fprintf('-----------------------\n');
    % Sample support points.
    [Pts] = SampleSupportPoint(num_pts, options_pt);
    % Sample pressure distribution.
    [Pds] = AssignPressure(Pts, options_pd);
    Pts = Pts';
    % Compute center of mass.
    Pts_com = Pts * Pds;
    % Shift coordinate so that com is the point of origin. 
    Pts = bsxfun(@minus, Pts, Pts_com);
    
    exp_record.Pts{ind_eval} = Pts;
    exp_record.Pds{ind_eval} = Pds;
    
    % Sample F,V pairs.
    num_facet_pts = ceil(r_facet * (num_cors / 2) / num_pts);
    num_other_pts = ceil((1 - r_facet) * (num_cors / 2));
    CORs = GenerateRandomCORs3(Pts, num_other_pts, num_facet_pts);
    [F, V] = GenFVPairsFromPD(Pts, Pds, CORs);
    pho = 1;
    F = F';
    [V, F] = NormalizeForceAndVelocities(V, F, pho);
    mean(sqrt(sum(F.^2, 2)));
    %h = figure; VisualizeForceVelPairs(F', V', h);
    % Split out test data.
    [V_train_val, V_test, F_train_val, F_test] = SplitTrainTestData(V, F, r_train);
    test_data.V = V_test;
    test_data.F = F_test;
    % Add noise(if any) to training forces data.
    noise_eps_f = noise.f;
    noise_eps_v = noise.v;
    F_train_val = F_train_val + noise_eps_f * randn(size(F_train_val));
    V_train_val = V_train_val + noise_eps_v * randn(size(V_train_val));
    V_train_val = UnitNormalize(V_train_val);
    
    % Among all data for training and validation, use 60% to construct
    % whole training set.
    [V_train_all, V_val, F_train_all, F_val] = SplitTrainTestData(V_train_val, F_train_val, 0.6);
    
    r = [0.15, 0.3, 0.5, 1.0];
    num_train_all = size(V_train_all,1);
    for ind_ratio = 1:1:length(r)
        % Use subset of the training data set as specified by r.
        V_train = V_train_all(1:floor(num_train_all * r(ind_ratio)), :);
        F_train = F_train_all(1:floor(num_train_all * r(ind_ratio)), :);
        fprintf('*********\nUse training size:%d\n', size(V_train,1));
        train_data.V = V_train;
        train_data.F = F_train;
        validation_data.V = V_val;
        validation_data.F = F_val;
        exp_record.train_data(ind_eval, ind_ratio) = train_data;
        exp_record.validation_data(ind_eval, ind_ratio) = validation_data;
        exp_record.test_data(ind_eval, ind_ratio) = test_data;
       
        [record] = MethodComparision(train_data, validation_data, test_data);
        for ind_method = 1:1:length(record.method)
            exp_record.err_test{ind_method}(ind_eval, ind_ratio) = record.err_test(ind_method);
            exp_record.err_train{ind_method}(ind_eval, ind_ratio) = record.err_train(ind_method);
            exp_record.err_validation{ind_method}(ind_eval, ind_ratio) = record.err_validation(ind_method);
        end    
    end
end


end

