function [h] = PlotBarsWithErrors(data, group_name, flag_legend)
if (nargin <=2)
    flag_legend = 0;
end
num_methods = 4;
h = figure;

for i = 1:1:num_methods
    alpha = 0.95;
    [mu, sigma, muci, sigmaci] = normfit(real(data{i}), 1 - alpha);
    MeanErr(i,:) = real(mu);
    BarWidth(i,:) = real(muci(2,:) - mu);
end
barwitherr(BarWidth', MeanErr');
if (flag_legend)
    legend('poly4cvx','poly4','quad', 'gp', 'Location','northeast');
end
set(gca,'XTickLabel', group_name);
xlabel('amount of training data');
ylabel('velocity alignment error(degree)');
end

