% Input:
% pos_2d: (3 * Np) (x,y,theta) in meter,meter and radian.
% vertices: (2 * Nv) counter-clockwise ordering of vertices.
function [h] = PlotFreeSliding(pos_2d, t, vertices, shift)
if nargin < 3
    vertices = [0,0;
                0.15,0;
                0,0.15;]';
end
if nargin < 4
    shift = [0;0];
end
num_vertices = size(vertices,2);
num_poses = size(pos_2d, 2);
h = figure;
hold on;
axis normal;
axis equal;
%cord_V_x = zeros(num_poses, num_vertices);
%cord_V_y = zeros(num_poses, num_vertices);
colors = ['r', 'g', 'b'];
ct = 1;
num_interval_plot = 50;
t_interval = (t(end) - t(1)) / num_interval_plot;
ind_interval = 1;
for i = 1:1:num_poses
    % Get plotting origin point coordinate.
    theta = pos_2d(3,i);
    % Plot object.
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    cord_xy = pos_2d(1:2,i) + R * shift;
    V = R * vertices;
    flag_draw_intermediate = 0;
    for j = 1:1:num_vertices
        x = cord_xy(1) + V(1,j);
        y = cord_xy(2) + V(2,j);
        cord_V_x(ct,j) = x;
        cord_V_y(ct,j) = y;
        if (j < num_vertices)
            nxt_ind_j = j + 1;
        else
            nxt_ind_j = 1;
        end
        nxt_x = cord_xy(1) + V(1,nxt_ind_j);
        nxt_y = cord_xy(2) + V(2,nxt_ind_j);
        if (i == 1)
        plot([x, nxt_x], [y, nxt_y], 'Color', colors(mod(j-1,3) + 1), ...
            'LineStyle', '-', 'LineWidth', 2);
        elseif (i == num_poses)
        plot([x, nxt_x], [y, nxt_y], 'Color', colors(mod(j-1,3) + 1), ...
            'LineStyle', '--', 'LineWidth', 2);   
        elseif (t(i+1) > (t_interval * ind_interval) && t(i) <= (t_interval * ind_interval))
            plot([x, nxt_x], [y, nxt_y], 'Color', colors(mod(j-1,3) + 1), ...
                'LineStyle', ':', 'LineWidth', 1);
            flag_draw_intermediate = 1;
        end
        %plot(x,y, 'Color', colors(mod(j-1,3) + 1), 'Marker', '^', 'MarkerSize', 5);   
    end
    if (flag_draw_intermediate)
        ind_interval = ind_interval + 1;
    end
    ct = ct + 1;
end

x_min = min(min(cord_V_x));
x_max = max(max(cord_V_x));
y_min = min(min(cord_V_y));
y_max = max(max(cord_V_y));
margin_x = 0.025;
margin_y = 0.025;

%xlim([x_min-margin_x, x_max + margin_x]);
%ylim([y_min-margin_x, y_max + margin_y]);
% Plot point of origin trajectory.
cord_com = pos_2d(1:2,:);
plot(cord_com(1,:), cord_com(2,:), 'k.', 'MarkerSize', 5);

% Add marker to vertices.
for i = 1:1:num_vertices
    plot(cord_V_x(1,i), cord_V_y(1,i), 'Color', colors(mod(i-1,3) + 1), 'Marker', '^', 'MarkerSize', 8, 'LineStyle', 'none');
    plot(cord_V_x(end,i), cord_V_y(end,i), 'Color', colors(mod(i-1,3) + 1), 'Marker', 's', 'MarkerSize', 8, 'LineStyle', 'none');
    plot(cord_V_x(:,i), cord_V_y(:,i), 'Color', colors(mod(i-1,3) + 1), 'Marker', '.', 'MarkerSize', 5, 'LineStyle', '-');
end

end

