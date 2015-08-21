close all;
tic;

exp_para.num_evals = 50;
exp_para.num_data = 150;
exp_para.r_facet = 0.5;
exp_para.r_train = 0.5;
exp_para.noise.f = 0.1;
exp_para.noise.v = 0.1;

scenario = '3-points';
[exp_record_3pts_noisy] = RunSimulationScenario(scenario, exp_para);
toc;

exp_para.noise.f = 0;
exp_para.noise.v = 0;
[exp_record_3pts_noisefree] = RunSimulationScenario(scenario, exp_para);
toc;


scenario = '360-ring';
exp_para.r_facet = 0;

exp_para.noise.f = 0.1;
exp_para.noise.v = 0.1;
[exp_record_360_ring_noisy] = RunSimulationScenario(scenario, exp_para);
toc;


exp_para.noise.f = 0;
exp_para.noise.v = 0;
[exp_record_360_ring_noisefree] = RunSimulationScenario(scenario, ...
                                                  exp_para);
toc;

save('new_50_evals_3pts_ring.mat', 'exp_record_3pts_noisy', ...
     'exp_record_3pts_noisefree', 'exp_record_360_ring_noisy','exp_record_360_ring_noisefree');

