close all; 
% Extract all the fig files.
k = dir('*.fig');
filenames = {k.name}';
for i =1:1:length(filenames)
    file_name = filenames{i}(1:end-4);
    h = openfig(file_name);
    saveas(h, [file_name, '.eps'], 'epsc');
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
