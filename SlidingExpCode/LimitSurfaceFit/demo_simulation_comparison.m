close all;
% Two bar supports. 
options_pt.mode = 'rim';
options_pt.range = 1;
num_pts = 2;
options_pd.mode = 'random';
num_evals = 10;
num_data = 200;
r_facet = 0.5;
r_train = 0.5;
noise.f = 0.05;
noise.v = 0.05;
[exp_record_bar] = MultiEval2(num_evals, num_pts, num_data, options_pt, options_pd, r_facet, r_train)

num_methods = 4;
%plot graph.
colors = ['r','g','b','k'];
figure;
hold on;
for i = 1:1:num_methods
    p = errorbar(1:1:5, mean(exp_record_bar.err_test{i}), std(exp_record_bar.err_test{i}), 'Color', colors(i));
end