load results_lc;  % load result_methods cell array.
index_lc = 1;


pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 0.05;
mu = 1.0;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;
unit_scale = 1000;

num_gen_cors = 200;
range_cor = 4;
rot_angle = 10; % in degree.
moveclose_dist = 0.5; % in mm. 
ContactInfo.approach_vectors = [-0.707107;-0.707107;0];
ContactInfo.push_points = [75;75;0];

[push_actions] = GenerateRandomPushCORActionsGivenPushLocations(ContactInfo, num_gen_cors, pho * unit_scale, range_cor, rot_angle, moveclose_dist);
[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);
[push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale);

eps_norm = 0:0.001:0.25;
eps_threshold = 0.01;
eps_threshold_index = int32(ceil((eps_threshold - eps_norm(1)) / (eps_norm(2) - eps_norm(1))));

results = cell(4,1);
% cvx_poly4.
fprintf('Poly4CVX\n');
lc_coeffs_poly4_cvx = result_methods{index_lc}.coeffs{1};
[stable_pred_poly4_cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm);

flag_stable_threshold_poly4_cvx = stable_pred_poly4_cvx.flag_stable_pred{eps_threshold_index}; 
% Draw the stable prediction results.
h_polycvx = figure;
DrawStableCOR(push_actions.push_points, push_actions.cors, flag_stable_threshold_poly4_cvx, h_polycvx)