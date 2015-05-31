% F, V: 3*N. 
% Assume V is normalized velocity.
function [fig] = VisualizeForceVelPairs(F, V)
fig = figure;
numF = size(F,2);
numV = size(V,2);
avg_f_norm = mean(sqrt(sum(F.^2)));
l = avg_f_norm * 0.1;
assert(numF == numV);
% Plot Forces as red dots.
plot3(F(1,:), F(2,:), F(3,:), 'r*', 'Markersize', 6);
hold on;
V = V * l;
%Plot Velocities as arrows from force points.
for i = 1:1:numF
    quiver3(F(1,i), F(2,i), F(3,i), V(1,i), V(2,i), V(3,i), 'b-');
end
% Something weird about arrow size.
%quiver3(F(1,:)', F(2,:)', F(3,:)', V(1,:)', V(2,:)', V(3,:)', 'b-');
end

