clear all;
close all;
rng(1);
options_pt.mode = 'polygon'; r = 5; options_pt.vertices = [-r,-r; -r,r; r,r; r,-r];
%options_pt.mode = 'Circle'; options_pt.range = 50; 
%options_pt.mode = 'polygon'; options_pt.vertices = [0,0;0,1;1,0];
options_pd.mode = 'random';

num_evals = 5;

num_pts = 3;
facet_points = 50;
num_cors = 150;

r_train = 0.1;

noise.f = 0;
noise.v = 0;
flag_use_dir = 1;
[ err_angles_train, err_angles_test, info ] = MultiEvaluation(num_evals, num_pts, num_cors, facet_points, r_train, noise, options_pt, options_pd, flag_use_dir);
mean(err_angles_train, 1)
std(err_angles_train,1)
mean(err_angles_test, 1)
std(err_angles_test,1)