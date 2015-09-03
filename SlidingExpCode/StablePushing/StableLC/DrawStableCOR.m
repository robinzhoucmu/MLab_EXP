% push_points: 3*n;
% cors: 3*m.
function [] = DrawStableCOR(pt_contacts, cors, flag_stable, h)
figure(h); 
axis equal;
hold on;
% Draw triangle work object.
plot([0,150,0,0], [0,0,150,0], 'o-g', 'LineWidth', 1.5);
plot(pt_contacts(1,:), pt_contacts(2,:), 'b*', 'MarkerSize', 8);
for i = 1:1:size(cors, 2)
    if flag_stable(i)
        color = 'r';
    else 
        color = 'k';
    end
    plot(cors(1,i), cors(2,i), 'Marker', '.', 'MarkerEdgeColor', color);
end
end

