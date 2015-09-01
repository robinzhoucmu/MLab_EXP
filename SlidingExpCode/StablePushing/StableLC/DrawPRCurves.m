function [] = DrawPRCurves(results, h)
figure(h);
hold on;
num_curves = length(results);
marker_type = ['o', 'x', 'd', 's'];
marker_size = [5,6,7,8];
colors = ['r','g','b','k'];
line_styles = ['--', '-', '-.', '-'];
line_width = [2, 1.5, 2, 1.5];
for i = 1:1:num_curves
    recall = results{i}(:,2);
    precision = results{i}(:,1);
    plot(recall, precision, 'Color', colors(i), 'Marker', marker_type(i), 'MarkerSize', marker_size(i), ...
         'LineStyle',line_styles(i), 'LineWidth', line_width(i));
end

end

