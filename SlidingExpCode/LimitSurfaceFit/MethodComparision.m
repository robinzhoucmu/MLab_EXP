function [record] = MethodComparision(train_data, validation_data, test_data)
if (nargin < 3)
    has_test_data = 0;
else
    has_test_data = 1;
end
record.method = {'poly4_convex', 'poly4_plain', 'quadratic' 'gp'};
% record deviation angle.
record.err_train = zeros(4,1);
record.err_validation = zeros(4,1);
record.err_test = zeros(4,1);

F_train = train_data.F;
F_train_dir = UnitNormalize(F_train);
V_train = train_data.V;
F_valid = validation_data.F;
F_valid_dir = UnitNormalize(F_valid);
V_valid = validation_data.V;
record.F_train = F_train;
record.V_train = V_train;

if (has_test_data)
    F_test = test_data.F;
    F_test_dir = UnitNormalize(F_test);
    V_test = test_data.V;
    record.F_test = F_test;
    record.V_test = V_test;
end

num_methods = 4;
for i = 1:1:num_methods
    %fprintf('Method:%s\n', record.method{i});
    if (i == 1)
        % Poly4.
        options.flag_convex = 1;
        options.method = 'poly4';
        options.flag_dir = 0;
    elseif (i == 2)
        % Poly4 without convexity.
        options.flag_convex = 0;
        options.method = 'poly4';
        options.flag_dir = 0;
    elseif (i== 3)
        options.flag_convex = 1;
        options.method = 'quadratic';
        options.flag_dir = 0;
    else
        options.method = 'gp';
    end    
    %h = figure; VisualizeForceVelPairs(F_train', V_train', h);

    [info] = CrossValidationSearchParameters(F_train, V_train, F_valid, V_valid, options);

    record.err_train(i) = info.train_dev_angle_record;
    record.err_validation(i) = info.dev_angle;
    record.coeffs{i} = info.coeffs;
    if (has_test_data)
        if (i==1 || i== 2)
            [err, dev_angle] = EvaluatePoly4Predictor(F_test, V_test, info.coeffs);
        elseif (i==3)    
            [err, dev_angle] = EvaluateLinearPredictor(F_test, V_test, info.coeffs);
        else
            F_train_gp = [F_train_dir; -F_train_dir];
            V_train_gp = [V_train; -V_train];
            [hyp, dev_angle_train, dev_angle] = GP_Fitting(F_train_gp, V_train_gp, F_test_dir, V_test, info.coeffs);
        end
        record.err_test(i) = dev_angle;
        fprintf('%s, err:%f\n', record.method{i}, dev_angle);
    end
end

end

