close all; 
% Extract all the fig files.
k = dir('*.fig');
filenames = {k.name}';
<<<<<<< HEAD
font_size = 18.5;
=======
font_size = 18;
>>>>>>> 90c3f3600ff451db042443bb9c5eb7c196a1bbb3
for i =1:1:length(filenames)
    file_name = filenames{i}(1:end-4);
    h = openfig(file_name);
    xlabel('F_x', 'FontSize', font_size);ylabel('F_y', 'FontSize', font_size);zlabel('F_z', 'FontSize', font_size);
    set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
    set(findall(gcf,'-property','FontSize'),'FontSize', font_size);
    %pause;
    save2pdf(file_name, gcf, 200);
    %export_fig(gcf, file_name, '-eps');
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
