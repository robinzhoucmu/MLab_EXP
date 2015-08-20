function [h1,h2] = PlotTestTrainErrorBar(r, exp_record)
num_methods = 4;
%plot graph.
colors = ['r','g','b','k'];
h1 = figure;
%title('testing error');
hold on;
%r = [0.15, 0.3, 0.5, 1.0];
xlabel('amount of data(percentage)');
xlim([10,105]);
ylabel('velocity alignment error(degree)');
for i = 1:1:num_methods
    [mu, sigma, muci, sigmaci] = normfit(real(exp_record.err_test{i}), 0.05)
    p = errorbar(r, mu, muci(2,:)-mu, ...
        'Color', colors(i), 'Marker', 'o', 'MarkerSize', 6, 'LineStyle','--');
end
legend('poly4-cvx','poly4-plain','quadratic', 'gp');

h2 = figure;
%title('training error');
hold on;
for i = 1:1:num_methods
    [mu, sigma, muci, sigmaci] = normfit(real(exp_record.err_train{i}), 0.05);
    p = errorbar(r, mu, muci(2,:)-mu, ...
        'Color', colors(i), 'Marker', 'o', 'MarkerSize', 6, 'LineStyle','--');
end
legend('poly4-cvx','poly4-plain','quadratic', 'gp');


end

