function [record] = MethodComparision(train_data, validation_data, test_data)
record.method = {'poly4_convex', 'quadratic', 'poly4_plain', 'GP'};
% record deviation angle.
record.err_train = zeros(4,1);
record.err_validation = zeros(4,1);
record.err_test = zeros(4,1);

F_train = train_data.F;
V_train = train_data.V;
F_valid = validation_data.F;
V_valid = validation_data.V;
F_test = test_data.F;
V_test = test_data.V;

% Poly4.
ind_poly4_convex = 1;

options.flag_convex = 1;
options.method = 'poly4';
options.flag_dir = 0;
[info] = CrossValidationSearchParameters(F_train, V_train, F_valid, V_valid, options);

record.err_train(ind_poly4_convex) = info.train_dev_angle_record;
record.err_validation(ind_poly4_convex) = info.dev_angle;
record.coeffs{ind_poly4_convex} = info.coeffs;

[err, dev_angle] = EvaluatePoly4Predictor(F_test, V_test, info.coeffs);
record.err_test(ind_poly4_convex) = dev_angle;


end

