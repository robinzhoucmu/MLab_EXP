% Script to generate a stable pushing plot.
% Use center of mass as point of origin. Long edge is the x-axis.

h = figure;
hold on;
%axis equal;
axis([-1000 1000 -1000 1000]);
%axis manual;
%xlim([-400 400]);
%ylim([-400 400]);
% Find the rotation matrix that represents the lower-left corner frame in
% com frame.
R = [-sqrt(2)/2, sqrt(2)/2 ;
     -sqrt(2)/2, -sqrt(2)/2;];
t = [0;50*sqrt(2)];
%p1 = [-150/sqrt(2), -25*sqrt(2)]; p2 = [150/sqrt(2), -25*sqrt(2)]; p3 = [0, 50*sqrt(2)];
tri_p = bsxfun(@plus, R * [0, 150,0;0,0, 150] , t);

for ind_tri_line = 1:1:3
    tri_line{ind_tri_line} = createLine(tri_p(:, ind_tri_line)', tri_p(:, mod(ind_tri_line,3) + 1)');
    drawEdge(tri_p(:, ind_tri_line)', tri_p(:, mod(ind_tri_line,3) + 1)', 'LineWidth', 2, 'Color', 'k');
end

% Plot the COM. 
plot([0;0], 'Marker', 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'y');

% Plot the fingers. 
dir_edge = [sqrt(2)/2; -sqrt(2)/2];
pos_fingers(:,1) = [75;75] + dir_edge * 25;
pos_fingers(:,2) = [75;75] - dir_edge * 25;
pos_fingers = bsxfun(@plus, R * pos_fingers , t);
plot(pos_fingers(1,:), pos_fingers(2,:), 'Marker', 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

% Draw the friction cone.
fc_edges = [-sqrt(2)/2, sqrt(2)/2; sqrt(2)/2, sqrt(2)/2];
fc_len = 35;
v_fc_edges = fc_edges * fc_len;
arrow_len = 20;
arrow_width = 10;
drawArrow([pos_fingers(:,1)', v_fc_edges(:,1)' + pos_fingers(:,1)'], arrow_len, arrow_width, .5);
drawArrow([pos_fingers(:,1)', v_fc_edges(:,2)' + pos_fingers(:,1)'], arrow_len, arrow_width, .5);
drawArrow([pos_fingers(:,2)', v_fc_edges(:,1)' + pos_fingers(:,2)'], arrow_len, arrow_width, .5);
drawArrow([pos_fingers(:,2)', v_fc_edges(:,2)' + pos_fingers(:,2)'], arrow_len, arrow_width, .5);

% bound at left point on the long edge. 
drawRay([tri_p(:,2)',-sqrt(2)/2, sqrt(2)/2]);
drawRay([tri_p(:,2)',-sqrt(2)/2, -sqrt(2)/2]);
% bound at right point on the long edge.
drawRay([tri_p(:,3)',sqrt(2)/2, sqrt(2)/2]);
drawRay([tri_p(:,3)',sqrt(2)/2, -sqrt(2)/2]);

r = sqrt((25*sqrt(2))^2 + (75*sqrt(2))^2);
p = sqrt(25^2 + (25*sqrt(2))^2);
d = r^2 / p;

l_f1 = createLine(pos_fingers(:,1)', [0;0]');
l_f2 = createLine(pos_fingers(:,2)', [0;0]');
% Draw bisection line. 
bisec_l_f1 = orthogonalLine(l_f1, pos_fingers(:,1)'/2);
bisec_l_f2 = orthogonalLine(l_f2, pos_fingers(:,2)'/2);
line_width_bound = 0.25;
color_line_bound = 'g';
drawLine(bisec_l_f1, 'Color', color_line_bound, 'LineWidth', line_width_bound);
drawLine(bisec_l_f2, 'Color', color_line_bound, 'LineWidth', line_width_bound);

% Draw dual bisection line.
dir_f1 = -pos_fingers(:,1)/norm(pos_fingers(:,1));
dir_f2 = -pos_fingers(:,2)/norm(pos_fingers(:,2));
dual_bisec_l_f1 = orthogonalLine(l_f1, dir_f1'*d);
dual_bisec_l_f2 = orthogonalLine(l_f2, dir_f2'*d);
drawLine(dual_bisec_l_f1, 'Color', color_line_bound, 'LineWidth', line_width_bound);
drawLine(dual_bisec_l_f2, 'Color', color_line_bound, 'LineWidth', line_width_bound);

%------------------------------------------------------------------------%
load results_lc;  

index_lc = 3;
rng(50)  %
pho = 0.05;
% Transformation from lower left corner to com. 
trans = [50;50;0];
R_tf = [-sqrt(2)/2,-sqrt(2)/2, 0;
        sqrt(2)/2, -sqrt(2)/2, 0;
        0, 0, 1];
H_tf = [eye(3,3), trans;
        0,0,0,1];
fingers_width = 50;
mu = 1;
R_tool_point = [sqrt(2)/2, sqrt(2)/2;
                sqrt(2)/2, -sqrt(2)/2]';
R_tool_two_points = -R_tool_point;
unit_scale = 1000;

num_gen_cors = 3000;
range_cor = 10;
rot_angle = 20; % in degree.
moveclose_dist = 0; % in mm. 
% One push scenario.
ContactInfo.approach_vectors = [-0.707107;
                                -0.707107;
                                0];
ContactInfo.push_points = [75;
                           75;
                           0];

[push_actions] = GenerateRandomPushCORActionsGivenPushLocations(ContactInfo, num_gen_cors, pho * unit_scale, range_cor, rot_angle, moveclose_dist);
[pt_contacts, pt_outward_normals] = ExtractPushContacts(push_actions, fingers_width, H_tf, unit_scale);
[push_vels] = ComputePushVelGivenPushActions(push_actions, H_tf, pho,  unit_scale);

eps_norm = 1e-5;
eps_threshold_index = 1;

%--------------------------------------------------%
% cvx_poly4.
fprintf('Poly4CVX\n');
lc_coeffs_poly4_cvx = result_methods{index_lc}.coeffs{1};
[stable_pred_poly4_cvx] = PredictTwoPointsStable(...
     push_vels, pt_contacts, pt_outward_normals, mu, pho, lc_coeffs_poly4_cvx, 'poly4', eps_norm);

flag_stable = stable_pred_poly4_cvx.flag_stable_pred{eps_threshold_index}; 

trans = [50;50;0];
R_tf_2 = [-sqrt(2)/2,-sqrt(2)/2, 0;
        sqrt(2)/2, -sqrt(2)/2, 0;
        0, 0, 1];
H_tf_2 = [R_tf_2, trans;
        0,0,0,1];

cors = H_tf_2 \ [push_actions.cors;ones(1, size(push_actions.cors,2))];
for i = 1:1:size(cors, 2)
    if flag_stable(i)
        color = 'r';
    else 
        color = [0.5 0.5 0.5];
    end
    plot(cors(1,i), cors(2,i), 'Marker', '.', 'MarkerEdgeColor', color, 'MarkerSize', 5);
end
axis_length = 450;
axis([-axis_length axis_length -axis_length axis_length]);


