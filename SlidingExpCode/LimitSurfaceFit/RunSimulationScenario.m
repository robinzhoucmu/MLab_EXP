function [ exp_record ] = RunSimulationScenario(scenario, exp_para)
num_evals = exp_para.num_evals;
num_data = exp_para.num_data;
noise = exp_para.noise;
r_facet = exp_para.r_facet;
r_train = exp_para.r_train;

if strcmp(scenario,'2-bar')
    options_pt.hyper_mode = 'random';
    options_pt.mode = 'rim';
    options_pt.range = 1;
    num_pts = 2;
    options_pd.mode = 'random';

    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)
elseif strcmp(scenario,'3-points')
    options_pt.hyper_mode = 'random';
    options_pt.mode = 'rim';
    options_pt.range = 1;
    num_pts = 3;
    options_pd.mode = 'random';
    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)

elseif strcmp(scenario,'360-ring')
    options_pt.hyper_mode = 'grid';
    options_pt.mode = 'rim'
    options_pt.range = 1;
    num_pts = 360;
    options_pd.mode = 'uniform';
    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)

elseif strcmp(scenario, 'square')
    options_pt.hyper_mode = 'grid';
    options_pt.mode = 'polygon';
    options_pt.vertices = [-0.5, -0.5, 0.5, 0.5; -0.5, 0.5, 0.5, -0.5]';
    num_pts = 400;
    options_pd.mode = 'uniform';
    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)
end



end

