clear all; close all;
%rng(1);

%log_file_name = 'SensorLogs/10_90_10_10_30_130/exp_08_15_50.txt';
%log_file_name = 'SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';
%log_file_name = 'SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
%log_file_name = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';

%log_files_set{1} = 'SensorLogs/wood_patch/exp_08_17_0839_50.txt';
log_files_set{1} = 'SensorLogs/wood_10_130_10_10_10_130/exp_08_17_50.txt';
log_files_set{2} = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
log_files_set{3} = 'SensorLogs/wood_10_90_10_10_30_130/exp_08_18_1100_50.txt';
log_files_set{4} = 'SensorLogs/wood_30_63.33_43.33_30_43.33_63.33/exp_08_18_1330_50.txt';
%H_tf = eye(4,4);
trans = [50;50;0];
H_tf = [eye(3,3), trans;
      0,0,0,1];
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';


% Parameters for trianglular block.         
Tri_mass = 1.518;
% Black board.
mu_f_blk = 4.2 / (Tri_mass * 9.8);
% Wood board.
mu_f_wood = 5.0 / (Tri_mass * 9.8);
Tri_com = [0.15/3; 0.15/3];
Tri_pho = 0.05;

% support Points
% Compute the patch support points.
options_pt.hyper_mode = 'grid';
options_pt.mode = 'polygon';
options_pt.vertices = [0,0.15,0;0,0,0.15]';
num_pts = 400;
Tri_pts_patch = GridSupportPoint(num_pts, options_pt)';

%Tri_pts_set{1} = Tri_pts_patch;
Tri_pts_set{1} = [0.01, 0.01, 0.13; 0.01,0.13,0.01];
Tri_pts_set{2} = [0.03    0.09    0.03; 0.03    0.03    0.09];
Tri_pts_set{3} = [0.01, 0.09,0.01;0.01,0.03,0.13];
Tri_pts_set{4} = [0.03, 0.06333, 0.04333; 0.03, 0.04333, 0.06333];

num_evals = 16;
num_cors = 603;
r_facet = 0.5;
unit_scale = 1000;
tic;
[exp_record_set, sim_record_set] = ...
    MultiLogsEval(log_files_set, num_evals, Tri_pts_set, Tri_mass, Tri_com, mu_f_wood, Tri_pho, R_tool, H_tf, num_cors, r_facet, unit_scale);
toc;