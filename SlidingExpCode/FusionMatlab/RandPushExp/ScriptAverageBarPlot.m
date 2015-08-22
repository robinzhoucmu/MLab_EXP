% Scripts to generate average plots.
load 15_evals_exp_sim_compare.mat;
num_logs = length(exp_record_set);
num_methods = 4;
num_train_splits = 4;

% Exp test performance bar.
err_test_all_exp = {};
for ind_method = 1:1:4
    data = zeros(num_evals * num_logs, num_train_splits);
    for i = 1:1:num_logs
        data((i-1)*num_evals+1:i*num_evals, :) = exp_record_set{i}.err_test{ind_method};
    end
    err_test_all_exp{ind_method} = data;
end
 group_name = {'5', '10', '20', '30'};
 h_exp = PlotBarsWithErrors(err_test_all_exp, group_name, 1);

 
 % Sim test performance bar.
err_test_all_sim = {};
for ind_method = 1:1:4
    data = zeros(num_evals * num_logs, num_train_splits);
    for i = 1:1:num_logs
        data((i-1)*num_evals+1:i*num_evals, :) = sim_record_set{i}.err_test{ind_method};
    end
    err_test_all_sim{ind_method} = data;
end
 group_name = {'5', '10', '20', '30'};
 h_sim = PlotBarsWithErrors(err_test_all_sim, group_name, 1);