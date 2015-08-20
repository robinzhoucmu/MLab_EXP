function [h] = PlotBarsWithErrors(exp_record)
num_methods = 4;
h = figure;
xlabel('amount of data(percentage)');
ylabel('velocity alignment error(degree)');
data = exp_record.err_test;
for i = 1:1:num_methods
    alpha = 0.95;
    [mu, sigma, muci, sigmaci] = normfit(data{i}, 1 - alpha);
    MeanErr(i,:) = real(mu);
    BarWidth(i,:) = real(muci(2,:) - mu);
end
barwitherr(BarWidth', MeanErr');
legend('poly4cvx','poly4','quad', 'gp');
set(gca,'XTickLabel',{'15%', '30%', '50%', '100%'})
end

