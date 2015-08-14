function [ exp_record ] = RunSimulationScenario(scenario, exp_para)
num_evals = exp_para.num_evals;
num_data = exp_para.num_data;
noise = exp_para.noise;
r_facet = exp_para.r_facet;
r_train = exp_para.r_train;

if strcmp(scenario,'2-bar')
    options_pt.mode = 'rim';
    options_pt.range = 1;
    num_pts = 2;
    options_pd.mode = 'random';

    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)
elseif strcmp(scenario,'3-points')
    options_pt.mode = 'rim';
    options_pt.range = 1;
    num_pts = 3;
    options_pd.mode = 'random';
    [exp_record] = MultiEval2(num_evals, num_pts, num_data, options_pt, ...
                                  options_pd, r_facet, r_train, noise)
else
    
end



end

