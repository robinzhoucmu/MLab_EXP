load results_lc;  % load result_methods cell array.
index_lc = 3;
rng(1);

pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 50;
mu = 0.95;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;
unit_scale = 1000;

num_gen_cors = 600;
range_cor = 2;
rot_angle = 20; % in degree.
moveclose_dist = 0.5; % in mm. 
ContactInfo.approach_vectors = [-0.707107;-0.707107;0];
ContactInfo.push_points = [75;75;0];

[push_actions] = GenerateRandomPushCORActionsGivenPushLocations(ContactInfo, num_gen_cors, pho * unit_scale, range_cor, rot_angle, moveclose_dist);
[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);
[push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale);

% eps_norm = 0:0.0005:0.01;
% eps_threshold = 0.0025;
% eps_threshold_index = int32(ceil((eps_threshold - eps_norm(1)) / (eps_norm(2) - eps_norm(1))));
eps_norm = 0.0001;
eps_threshold_index = 1;

results = cell(4,1);
% cvx_poly4.
fprintf('Poly4CVX\n');
lc_coeffs_poly4_cvx = result_methods{index_lc}.coeffs{1};
[stable_pred_poly4_cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm);

flag_stable_threshold_poly4_cvx = stable_pred_poly4_cvx.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_polycvx = figure;
finger_pts = bsxfun(@plus, pt_contacts{1} * unit_scale, trans(1:2));
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_poly4_cvx, h_polycvx)

fprintf('quadratic\n');
lc_coeffs_quad = result_methods{index_lc}.coeffs{3};
[stable_pred_quad] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_quad, 'quadratic', eps_norm);

flag_stable_threshold_quad = stable_pred_quad.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_quad = figure;
finger_pts = bsxfun(@plus, pt_contacts{1} * unit_scale, trans(1:2));
DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_quad, h_quad)

% fprintf('gaussianprocess\n');
% gp_record.F_train = bsxfun(@rdivide, train_data.F, sqrt(sum(train_data.F.^2)));
% gp_record.V_train = train_data.V;
% gp_record.coeffs = result_methods.coeffs{4};
% lc_coeffs_gp = result_methods{index_lc}.coeffs{4};
% [stable_pred_gp] = PredictTwoPointsStable(...
%      push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_gp, 'gp', eps_norm);
% 
% flag_stable_threshold_gp = stable_pred_gp.flag_stable_pred{eps_threshold_index}; 
% % Draw the stable prediction results.
% h_gp = figure;
% finger_pts = bsxfun(@plus, pt_contacts{1} * unit_scale, trans(1:2));
% DrawStableCOR(finger_pts, push_actions.cors, flag_stable_threshold_gp, h_gp)


flag_diff = xor(flag_stable_threshold_poly4_cvx, flag_stable_threshold_quad);
ind_diff = find(flag_diff == 1);
length(ind_diff)
pred_diff_poly4 = flag_stable_threshold_poly4_cvx(ind_diff);
pred_diff_quad = flag_stable_threshold_quad(ind_diff);
file_name = 'push_action_diff.txt';
WritePushActionToDisk(push_actions, file_name, flag_diff);
