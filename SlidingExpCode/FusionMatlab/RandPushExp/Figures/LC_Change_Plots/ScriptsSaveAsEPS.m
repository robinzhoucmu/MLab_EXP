close all; 
% Extract all the fig files.
k = dir('InPaper/*.fig');
filenames = {k.name}';
for i =1:1:length(filenames)
    file_name = filenames{i}(1:end-4);
    h = openfig(file_name);
    %xlabel('F_x', 'FontSize', 10);ylabel('F_y', 'FontSize', 10);zlabel('F_z', 'FontSize', 10);
    %et(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
    %set(findall(gcf,'-property','FontSize'),'FontSize', 10);
    pause;
    export_fig(gcf, file_name, '-eps');
    %saveas(h, [file_name, '.eps'], 'eps');
    %print(gcf, '-depsc', '-painters', [file_name, '.eps']);
    close all;
end

% file_name_fig = '50_10_ideal'; 
% h = openfig(file_name_fig); 
% saveas(h,[file_name_fig,'.eps'],'epsc');
% 
% file_name_fig = '50_10_10train10val_poly4'; 
% h = openfig(file_name_fig); 
% saveas(h,[file_name_fig,'.eps'],'epsc');
% 
% file_name_fig = '50_10_10train10val_poly4plain'; 
% h = openfig(file_name_fig); 
% saveas(h,[file_name_fig,'.eps'],'epsc');
% 
% file_name_fig = '50_10_10train10val_quadratic'; 
% h = openfig(file_name_fig); 
% saveas(h,[file_name_fig,'.eps'],'epsc');
