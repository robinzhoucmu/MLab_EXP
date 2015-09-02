% push_points: 3*n;
% cors: 3*m.
function [] = DrawStableCOR(push_points, cors, flag_stable, h)
figure(h); 
hold on;
% Draw triangle work object.
plot([0,150,0,0], [0,0,150,0], 'o-');
plot(push_points(1,:), push_points(2,:), '^', 'MarkerSize', 5);
for i = 1:1:size(cors, 2)
    if flag_stable(i)
        color = 'r';
    else 
        color = 'k';
    end
    plot(cors(1,i), cors(2,i), 'Marker', '*', 'MarkerEdgeColor', color);
end
end

