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
unit = 1000;
num_vertices = size(vertices,2);
num_poses = size(pos_2d, 2);
h = figure;
hold on;
axis normal;
axis equal;
%axis equal;
%cord_V_x = zeros(num_poses, num_vertices);
%cord_V_y = zeros(num_poses, num_vertices);
colors = ['r', 'g', 'b'];
ct = 1;
num_interval_plot = 25;
t_interval = (t(end) - t(1)) / num_interval_plot;
ind_interval = 1;
for i = 1:1:num_poses
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
            plot([x, nxt_x] * unit, [y, nxt_y]*unit, 'Color', colors(mod(j-1,3) + 1), ...
                'LineStyle', '-', 'LineWidth', 3);
             cord_com = pos_2d(1:2,i);
             plot(cord_com(1,:) * unit, cord_com(2,:)*unit, 'k.', 'MarkerSize', 6);
        elseif (i == num_poses)
            %c = colors(mod(j-1,3) + 1);
            c = 'k';
            plot([x, nxt_x]*unit, [y, nxt_y]*unit, 'Color', c, ...
                'LineStyle', '-', 'LineWidth', 3);   
            elseif (t(i+1) > (t_interval * ind_interval) && t(i) <= (t_interval * ind_interval))
                plot([x, nxt_x]*unit, [y, nxt_y]*unit, 'Color', colors(mod(j-1,3) + 1), ...
                    'LineStyle', ':', 'LineWidth', 1.5);
                cord_com = pos_2d(1:2,i);
                plot(cord_com(1,:)*unit, cord_com(2,:)*unit, 'k.', 'MarkerSize', 7);
                flag_draw_intermediate = 1;
        end
        %plot(x,y, 'Color', colors(mod(j-1,3) + 1), 'Marker', '^', 'MarkerSize', 5);   
    end
    if (i>=num_poses  && i <= num_poses )
        %d = pos_2d(:, i+1)  - pos_2d(:, i);
        %v = d / norm(d);
        v = pos_2d(4:6, i);
        v(1:2) = R' * v(1:2);
        cor = [- v(2) / v(3); v(1) / v(3)]
        gl_cor = R * cor + pos_2d(1:2, i)
        local_cor = R' * (cor - pos_2d(1:2,i))
        plot(gl_cor(1) * unit, gl_cor(2) * unit, 'Marker', 'o'  ... 
            ,'MarkerFaceColor', 'm', 'MarkerSize', 8);
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
%cord_com = pos_2d(1:2,:);
%plot(cord_com(1,:), cord_com(2,:), 'k.', 'MarkerSize', 6);

% Add marker to vertices.
for i = 1:1:num_vertices
    plot(cord_V_x(1,i) * unit, cord_V_y(1,i) * unit, 'Color', colors(mod(i-1,3) + 1), 'Marker', '^', 'MarkerSize', 8, 'LineStyle', 'none');
    plot(cord_V_x(end,i) * unit, cord_V_y(end,i) * unit, 'Color', colors(mod(i-1,3) + 1), 'Marker', 's', 'MarkerSize', 8, 'LineStyle', 'none');
    plot(cord_V_x(:,i) * unit, cord_V_y(:,i) * unit, 'Color', colors(mod(i-1,3) + 1), 'Marker', '.', 'MarkerSize', 5, 'LineStyle', '-');
end
xlabel('X/mm', 'FontSize', 12);
ylabel('Y/mm', 'FontSize', 12);
xl = xlim;
yl = ylim;
margin = 15;
xlim([xl(1) - margin, xl(2) + margin]);
ylim([yl(1) - margin, yl(2) + margin]);
end

