function [multi_eval_results, multi_eval_accuracy] = MultiStableActionEval(log_file_poke, log_push_action, flag_gt, num_per_eval)
rng(1);
pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
unit_scale = 1000;   
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
Tri_com = [0.15/3; 0.15/3];

fingers_width = 50;
mu = 0.95;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;

v_ratio_train = [0.25, 0.5, 0.75];
for ind_ratio_train = 1:1:length(v_ratio_train)
ratio_train = v_ratio_train(ind_ratio_train);
    for ind_eval = 1:1:num_per_eval
    ind_record = ind_eval + (ind_ratio_train - 1) * num_per_eval;
    [record_log_poke] = ExtractFromLog(log_file_poke, pho, R_tool_point, H_tf, unit_scale);

    ratio_val = 0.2;
    [slider_velocities_train, slider_velocities_val, push_wrenches_train, push_wrenches_val] = ...
                    SplitTrainTestData(record_log_poke.slider_velocities, record_log_poke.push_wrenches, 1 - ratio_val);

    validation_data.V = slider_velocities_val;
    validation_data.F = push_wrenches_val;

    num_train_all = size(slider_velocities_train, 1);
    num_train = floor(num_train_all * ratio_train);
    index_train = randperm(num_train_all);
    fprintf('*********\nUse training size:%d\n', num_train);
    train_data.V = slider_velocities_train(index_train(1:num_train), :);
    train_data.F = push_wrenches_train(index_train(1:num_train), :);

    [result_methods] = MethodComparision(train_data, validation_data);

    %---------------------------
    [push_actions] = ParsePushActionLog(log_push_action);
    [pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);
    [push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale);

    eps_norm = eps;
    eps_threshold_index = 1;

    fprintf('Poly4CVX\n');
    lc_coeffs_poly4_cvx = result_methods.coeffs{1};
    [stable_pred_poly4_cvx] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm);
    multi_eval_results{ind_record}.pred_flag(:,1) = stable_pred_poly4_cvx.flag_stable_pred{eps_threshold_index}; 

    fprintf('Poly4\n');
    lc_coeffs_poly4 = result_methods.coeffs{2};
    [stable_pred_poly4] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4, 'poly4', eps_norm);

    multi_eval_results{ind_record}.pred_flag(:,2) = stable_pred_poly4.flag_stable_pred{eps_threshold_index}; 

    fprintf('quadratic\n');
    lc_coeffs_quad = result_methods.coeffs{3};
    [stable_pred_quad] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_quad, 'quadratic', eps_norm);

    multi_eval_results{ind_record}.pred_flag(:,3) = stable_pred_quad.flag_stable_pred{eps_threshold_index}; 

    lc_coeffs_gp = result_methods.coeffs{4};
    F_train = result_methods.F_train;
    V_train = result_methods.V_train;
    gp_record.F_train = bsxfun(@rdivide, F_train, sqrt(sum(F_train.^2)));
    gp_record.V_train = V_train;

    gp_record.coeffs = result_methods.coeffs{4};
    [stable_pred_gp] = PredictTwoPointsStable(...
         push_vels, pt_contacts, pt_outward_normals, mu, pho, gp_record, 'gp', eps_norm);

    multi_eval_results{ind_record}.pred_flag(:,4) = stable_pred_gp.flag_stable_pred{eps_threshold_index}; 
    for i = 1:1:4
        multi_eval_accuracy(ind_record, i) = length(find(multi_eval_results{ind_record}.pred_flag(:,i) == flag_gt)) / length(flag_gt); 
    end
    end
end
end

