% Evaluate on multiple logs.
% Input:
% log_files: cell array of multiple log files.
% num_evals: number of random evaluation.
% Tri_pts: cell array of support points.
% Tri_mass: mass of the object.
% mu_f: empirical coefficient of friction among surface and object support
% points.
% Tri_pho: radius of gyration of the object.
% R_tool: tool frame transformation w.r.t to robot tool frame.
% H_tf: object local frame transformation w.r.t to mocap rigid body frame.
% num_cors: number of COR points sampled from ideal LC.
% r_facet: ratio of points on the facet sampled from ideal LC.
% unit_scale: scale of measurement w.r.t to milimeter.
% Output:
% exp_record: cell array of record structure containing test performance 
% on experimental data (hold-out set from real sensor logs).
% sim_record: cell array of record structure containing simulation
% data(from ideal LC).

function [exp_record_set, sim_record_set] = MultiLogsEval(log_files, num_evals, Tri_pts_set, Tri_mass, Tri_com, mu_f, Tri_pho, R_tool, H_tf, num_cors, r_facet, unit_scale)
num_logs = length(log_files);
for ind_log = 1:num_logs   
    log_file_name = log_files{ind_log};
    Tri_V = [0,0.15,0;0,0,0.15];
    Tri_pts = Tri_pts_set{ind_log};
    num_pts = size(Tri_pts, 2);
    Tri_effective_mass = Tri_mass * mu_f;
    Tri_pts_cp = Tri_pts;
    Tri_pts = bsxfun(@minus, Tri_pts, Tri_com);
    if (num_pts == 3) 
        [Tri_pds] = GetObjParaFromSupportPts(Tri_pts, [0;0], Tri_effective_mass);
        h_tri = DrawTriangle(Tri_V, Tri_com, Tri_pts_cp, Tri_pds);
    else
        options_pd.mode = 'uniform';
        [Tri_pds] = AssignPressure(Tri_pts', options_pd);
        % Also remember to change r_facet.
        r_facet = 0;
    end

    [ record_log ] = ExtractFromLog( log_file_name, Tri_pho, R_tool, H_tf, unit_scale);

    % Sample from the ideal pressure distribution as test data. 
   
    num_facet_pts = ceil(r_facet * (num_cors / 2) / num_pts);
    num_other_pts = ceil((1 - r_facet) * (num_cors / 2));
    CORs = GenerateRandomCORs3(Tri_pts, num_other_pts, num_facet_pts);
    [F, bv] = GenFVPairsFromPD(Tri_pts, Tri_pds, CORs);

    % Change to row representation.
    F = F';
    pho = Tri_pho;
    [bv, F] = NormalizeForceAndVelocities(bv, F, pho);
    testsim_data.V = bv;
    testsim_data.F = F;
    exp_record = {};
    sim_record = {};
    for ind_eval = 1:1:num_evals
        fprintf('-----------------------\n');
        fprintf('percentage completed:%f\n', (ind_eval-1) * 100 / (num_evals * num_logs));
        fprintf('-----------------------\n');
        % Split out test data. 50*0.2 = 10;
        ratio_test = 0.2;
        [slider_velocities_train_val, slider_velocities_test, push_wrenches_train_val, push_wrenches_test] = ...
            SplitTrainTestData(record_log.slider_velocities, record_log.push_wrenches, 1 - ratio_test);
        testexp_data.V = slider_velocities_test;
        testexp_data.F = push_wrenches_test;

        % Split out validation data from experiment data. 
        % (1 - ratio_test) * num_data * ratio_val
        % 40*0.25 = 10;
        ratio_val = 0.25;
        [slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
             SplitTrainTestData(slider_velocities_train_val, push_wrenches_train_val, 1 - ratio_val);
        validation_data.V = slider_velocities_val;
        validation_data.F = push_wrenches_val;

        r = [1/3, 2/3, 1.0];
        num_methods = 4;
        num_train_all = size(slider_velocities_train, 1);
        for ind_ratio = 1:1:length(r)
            num_train = floor(num_train_all * r(ind_ratio));
            fprintf('*********\nUse training size:%d\n', num_train);
            train_data.V = slider_velocities_train(1:num_train, :);
            train_data.F = push_wrenches_train(1:num_train, :);
            % Evaluate on experimental test data.
            [record_exp] = MethodComparision(train_data, validation_data, testexp_data);
            for ind_method = 1:1:num_methods
                exp_record.err_test{ind_method}(ind_eval, ind_ratio) = record_exp.err_test(ind_method);
                exp_record.err_train{ind_method}(ind_eval, ind_ratio) = record_exp.err_train(ind_method);
                exp_record.err_validation{ind_method}(ind_eval, ind_ratio) = record_exp.err_validation(ind_method);
            end
            % Evaluate on simulation test data.
            [record_sim] = MethodComparision(train_data, validation_data, testsim_data);
            for ind_method = 1:1:num_methods
                sim_record.err_test{ind_method}(ind_eval, ind_ratio) = record_sim.err_test(ind_method);
                sim_record.err_train{ind_method}(ind_eval, ind_ratio) = record_sim.err_train(ind_method);
                sim_record.err_validation{ind_method}(ind_eval, ind_ratio) = record_sim.err_validation(ind_method);
            end

        end

    end
    toc;
    
    exp_record_set{ind_log} = exp_record;
    sim_record_set{ind_log} = sim_record;s
    
    group_name = {'10', '20', '30'};

    data_exp = exp_record.err_test;
    h_exp = PlotBarsWithErrors(data_exp, group_name);

    data_sim = sim_record.err_test;
    h_sim = PlotBarsWithErrors(data_sim, group_name);

end
end

