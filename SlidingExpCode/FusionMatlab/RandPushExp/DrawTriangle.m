% Input:
% Tri_V: 2*3; triangular vertices assuming lower left corner is at (0,0). 
% Tri_Pts: support points locations. Same coordinate system as Tri_V.
% Tri_Pds: support points pressure. 
function [h] = DrawTriangle(Tri_V, Tri_com, Tri_Pts, Tri_Pds)
h = figure;
hold on;
% Draw the triangle. 
num_v = size(Tri_V, 2);
for i = 1:1:num_v
  x = Tri_V(1, i);
  y = Tri_V(2, i);
  if (i < num_v)
        nxt_ind_i = i + 1;
  else
        nxt_ind_i = 1;
  end
  nxt_x = Tri_V(1,nxt_ind_i);
  nxt_y = Tri_V(2,nxt_ind_i);
  plot([x, nxt_x], [y, nxt_y], 'Color', 'b', ...
       'LineStyle', '-', 'LineWidth', 2);
end
% Draw a plus at the COM.
plot(Tri_com(1), Tri_com(2), 'r+', 'MarkerSize', 10);
% Draw the support points with size proportional to pressure.
tri_pds = Tri_Pds / sum(Tri_Pds);
num_pts = size(Tri_Pts,2);
for i = 1:1:num_pts
    plot(Tri_Pts(1,i), Tri_Pts(2,i), 'Marker', 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 20*tri_pds(i));
end
end

