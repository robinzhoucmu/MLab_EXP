function [h] = ImproveFigure(h)
figure(h);
grid on;
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
%set(gca, 'xtick', -1:0.5:16);
%set(gca, 'ytick', -360:120:360);
set(gca, 'Linewidth', 1);
x = get(gca, 'XLabel');
y = get(gca, 'YLabel');
set(findall(gcf,'-property','FontSize'),'FontSize', 14);
%set(x,'FontSize', 14);
%set(y,'FontSize', 14);
child = get(gca, 'Children')
for i=1:size(child,1)
    set(child(i),'Linewidth', 1);
end