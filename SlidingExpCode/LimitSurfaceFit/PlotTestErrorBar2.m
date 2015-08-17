% Plot noise-free and noisy test performance in one plot.
function [h] = PlotTestErrorBar2(x, exp_record_no_noise, exp_record_noisy)
num_methods = 4;
marker_type = ['o', 'x', 'd', 's'];
colors = ['r','g','b','k'];
h = figure;
hold on;
xlabel('amount of data(percentage)');
xlim([10,105]);
ylabel('velocity alignment error(degree)');
% Plot the noise-free error bars.
data = exp_record_no_noise.err_test;
for i = 1:1:num_methods
    alpha = 0.95;
    [mu, sigma, muci, sigmaci] = normfit(data{i}, 1 - alpha);
    p = errorbar(x, mu, muci(2,:) - mu, ...
                'Color', colors(i), 'Marker', marker_type(i), 'MarkerSize', 7, ...
                'LineStyle','--', 'LineWidth',1);
%     p_e = get(p, 'Children');
%     p_xdata = get(p_e(2), 'XData');
%     p_xdata(:) = p_xdata(:) - 0.1;
%     set(p_e(2), 'XData', p_xdata);
end
% Plot the noisy exp error bars.
marker_type = ['+', '*', '^', 'p'];
data = exp_record_noisy.err_test;
for i = 1:1:num_methods
    alpha = 0.95;
    [mu, sigma, muci, sigmaci] = normfit(data{i}, 1 - alpha);
    p = errorbar(x, mu, muci(2,:) - mu, ...
                'Color', colors(i), 'Marker', marker_type(i), 'MarkerSize', 7, ...
                'LineStyle','-', 'LineWidth',1);
end
legend('poly4cvx','poly4','quad', 'gp', 'poly4cvx-noisy','poly4-noisy','quad-noisy', 'gp-noisy');

set(findall(h,'-property','FontName'),'FontName','Times New Roman');


end

