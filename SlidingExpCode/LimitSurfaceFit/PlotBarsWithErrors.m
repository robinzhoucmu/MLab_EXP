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
    h_lgd = legend('Poly4cvx','Poly4','Quad', 'GP', 'Location','northeast');
    set(h_lgd, 'Position', [0.55, 0.7, 0.32, 0.23]);
end
set(gca,'XTickLabel', group_name);
xlabel('amount of training data');
ylabel('velocity alignment error(degree)');
end

