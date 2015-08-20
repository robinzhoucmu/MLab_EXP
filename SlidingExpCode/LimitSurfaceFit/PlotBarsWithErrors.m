function [h] = PlotBarsWithErrors(data, group_name)
num_methods = 4;
h = figure;

for i = 1:1:num_methods
    alpha = 0.95;
    [mu, sigma, muci, sigmaci] = normfit(real(data{i}), 1 - alpha);
    MeanErr(i,:) = real(mu);
    BarWidth(i,:) = real(muci(2,:) - mu);
end
barwitherr(BarWidth', MeanErr');
legend('poly4cvx','poly4','quad', 'gp', 'Location','northwest');
set(gca,'XTickLabel', group_name);
xlabel('amount of training data');
ylabel('velocity alignment error(degree)');
end

