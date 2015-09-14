% 150 * 0.5 = 75 data for testing.
% 75 * 0.4 = 30 data for validation.
% (75 - 30) * [0.15,0.3,0.5,1] = [7, 15, 22, 45]. 
close all;
clear all;

%load 50_evals_150_data_all.mat;
load new_50_evals_3pts_ring.mat;
group_name = {'7', '15', '22', '45'};

% 3pts noisy.
data = exp_record_3pts_noisy.err_test;
[h] = PlotBarsWithErrors(data, group_name, 1);
h = ImproveFigure(h);
file_name_fig = 'ExpRecord/BarPlotsNew/3pts_noisy2';
%savefig(h, file_name_fig);
print(h, file_name_fig, '-deps');

% 3pts noise-free.
data = exp_record_3pts_noisefree.err_test;
[h] = PlotBarsWithErrors(data, group_name,1);
h = ImproveFigure(h);
file_name_fig = 'ExpRecord/BarPlotsNew/3pts_noise_free2';
%savefig(h, file_name_fig);
print(h, file_name_fig, '-deps');

% % bar noisy.
% data = exp_record_bar_noisy.err_test;
% [h] = PlotBarsWithErrors(data, group_name,1);
% h = ImproveFigure(h);
% file_name_fig = 'ExpRecord/Figures/BarPlots/bar_noisy';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');
% 
% % bar noise-free
% data = exp_record_bar_noise_free.err_test;
% [h] = PlotBarsWithErrors(data, group_name,1);
% file_name_fig = 'ExpRecord/Figures/BarPlots/bar_noise_free';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');
% 
% % 10pts noisy.
% data = exp_record_10pts_noisy.err_test;
% [h] = PlotBarsWithErrors(data, group_name, 1);
% file_name_fig = 'ExpRecord/Figures/BarPlots/10pts_noisy';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');
% 
% % 10pts noise-free.
% data = exp_record_10pts_noise_free.err_test;
% [h] = PlotBarsWithErrors(data, group_name);
% file_name_fig = 'ExpRecord/Figures/BarPlots/10pts_noise_free';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');


% ring noisy.
data = exp_record_360_ring_noisy.err_test;
[h] = PlotBarsWithErrors(data, group_name, 1);
h = ImproveFigure(h);
file_name_fig = 'ExpRecord/BarPlotsNew/ring_noisy2';
%savefig(h, file_name_fig);
print(h, file_name_fig, '-deps');

% ring noise-free
data = exp_record_360_ring_noisefree.err_test;
[h] = PlotBarsWithErrors(data, group_name, 1);
% Adjust y axis to put in legend.
ylim([0,11]);
h = ImproveFigure(h);
file_name_fig = 'ExpRecord/BarPlotsNew/ring_noise_free2';
%savefig(h, file_name_fig);
print(h, file_name_fig, '-deps');

% % Square noisy.
% data = exp_record_square_noisy.err_test;
% [h] = PlotBarsWithErrors(data, group_name, 1);
% file_name_fig = 'ExpRecord/Figures/BarPlots/square_noisy';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');
% 
% % Square noise-free.
% data = exp_record_square_noise_free.err_test;
% [h] = PlotBarsWithErrors(data, group_name);
% file_name_fig = 'ExpRecord/Figures/BarPlots/square_free';
% savefig(h, file_name_fig);
% print(h, file_name_fig, '-deps');



