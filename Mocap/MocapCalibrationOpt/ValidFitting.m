function [meanErr] = ValidFitting(Pc, Pm, RotMatrices, R, t, v)
N = size(Pc,2);
Pr = zeros(3, N);
for i = 1:1:N
    Pr(:,i) = RotMatrices{i} * v + Pc(:,i);
end
Q = bsxfun(@plus, R * Pm, t);
dist = sqrt(sum((Q - Pr).^2));
meanErr = sum(dist)/ N;
figure;
plot(1:1:N, dist, 'o');
figure;
scatter3(Pr(1,:), Pr(2,:), Pr(3,:),'g*');
hold on;
scatter3(Q(1,:), Q(2,:), Q(3,:), 'r.');
end