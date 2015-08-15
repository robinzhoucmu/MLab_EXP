close all;
tic;

exp_para.num_evals = 50;
exp_para.num_data = 150;
exp_para.r_facet = 0.5;
exp_para.r_train = 0.5;
exp_para.noise.f = 0.1;
exp_para.noise.v = 0.1;

% scenario = '2-bar';
% [exp_record_bar] = RunSimulationScenario(scenario, exp_para);
% toc;
% [h1_bar,h2_bar] = PlotTestTrainErrorBar(exp_record_bar);
% 
% scenario = '3-points';
% [exp_record_3pts] = RunSimulationScenario(scenario, exp_para);
% toc;
% [h1_3pts,h2_3pts] = PlotTestTrainErrorBar(exp_record_3pts);
% 
% save('ExpRecord/50_evals_150_data_0f_0v', 'exp_record_bar', 'exp_record_3pts');

scenario = '10-points';
exp_para.noise.f = 0;
exp_para.noise.v = 0;
exp_para.r_facet = 0.5;
[exp_record_10pts] = RunSimulationScenario(scenario, exp_para);
toc;
[h1_10pts,h2_10pts] = PlotTestTrainErrorBar(exp_record_10pts);

exp_para.noise.f = 0.1;
exp_para.noise.v = 0.1;
[exp_record_10pts_noisy] = RunSimulationScenario(scenario, exp_para);
toc;
[h1_10pts_noisy,h2_10pts_noisy] = PlotTestTrainErrorBar(exp_record_10pts_noisy);
save('ExpRecord/10_pts_all_50_evals_150_data.mat', 'exp_record_10_pts', 'exp_record_10pts_noisy');




scenario = '360-ring';
%change to random sample without explictly on facet. 
exp_para.r_facet = 0;
[exp_record_360_ring] = RunSimulationScenario(scenario, exp_para);
toc;
[h1_360ring,h2_360ring] = PlotTestTrainErrorBar(exp_record_360_ring);


scenario = 'square';
exp_para.r_facet = 0;
[exp_record_square] = RunSimulationScenario(scenario, exp_para);
toc;
[h1_square,h2_square] = PlotTestTrainErrorBar(exp_record_square);

save('ExpRecord/50_evals_150_data_0.2f_0.2v_Grid', 'exp_record_360_ring', 'exp_record_square');


