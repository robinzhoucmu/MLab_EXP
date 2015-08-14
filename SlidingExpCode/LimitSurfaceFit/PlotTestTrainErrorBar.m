function [h1,h2] = PlotTestTrainErrorBar(exp_record)
num_methods = 4;
%plot graph.
colors = ['r','g','b','k'];
h1 = figure;
title('testing error');
hold on;
for i = 1:1:num_methods
    p = errorbar(1:1:size(exp_record.err_test{i}, 2), mean(exp_record.err_test{i}), std(exp_record.err_test{i}), ...
        'Color', colors(i), 'Marker', '*', 'MarkerSize', 6);
end
legend('poly4-cvx','poly4-plain','quadratic', 'gp');

h2 = figure;
title('training error');
hold on;
for i = 1:1:num_methods
    p = errorbar(1:1:size(exp_record.err_train{i}, 2), mean(exp_record.err_train{i}), std(exp_record.err_train{i}), ...
        'Color', colors(i), 'Marker', '*', 'MarkerSize', 6);
end
legend('poly4-cvx','poly4-plain','quadratic', 'gp');


end

