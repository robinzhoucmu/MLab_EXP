clear all;
close all;
rng(1);
%options_pt.mode = 'circle'; options_pt.range = 2; 
options_pt.mode = 'polygon'; options_pt.vertices = [0,0;0,1;1,0];
options_pd.mode = 'uniform';

num_evals = 10;

num_pts = 3;
facet_points = 20;
num_cors = 50;

r_train = 0.1;

noise.f = 0;
noise.v = 0;

[ err_angles_train, err_angles_test, info ] = MultiEvaluatation(num_evals, num_pts, num_cors, facet_points, r_train, noise, options_pt, options_pd);
mean(err_angles_train, 1)
mean(err_angles_test, 1)