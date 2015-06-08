clear all;
close all;
rng(1);
options_pt.mode = 'polygon'; r = 5; options_pt.vertices = [-r,-r; -r,r; r,r; r,-r];
%options_pt.mode = 'Circle'; options_pt.range = 50; 
%options_pt.mode = 'polygon'; options_pt.vertices = [0,0;0,1;1,0];
options_pd.mode = 'uniform';

num_evals = 10;

num_pts = 30;
facet_points = 5;
num_cors = 150;

r_train = 0.2;

noise.f = 0.15;
noise.v = 0.15;
flag_use_dir = 0;
[ err_angles_train, err_angles_test, info ] = MultiEvaluation(num_evals, num_pts, num_cors, facet_points, r_train, noise, options_pt, options_pd, flag_use_dir);
mean(err_angles_train, 1)
std(err_angles_train,1)
mean(err_angles_test, 1)
std(err_angles_test,1)