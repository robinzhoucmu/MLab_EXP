function [Pc, Pm, RotMatrices] = RemoveOutliers(Pc, Pm, RotMatrices, R, t, v, threshold)
N = size(Pc,2);
Pr = zeros(3, N);
for i = 1:1:N
    Pr(:,i) = RotMatrices{i} * v + Pc(:,i);
end
Q = bsxfun(@plus, R * Pm, t);
dist = sqrt(sum((Q - Pr).^2));

ind = dist > threshold;
disp('Removed ');
N - length(ind == 0)
Pc(:,ind) = [];
Pm(:,ind) = [];
RotMatrices(ind) = [];
end

