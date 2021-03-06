close all;
rng(1);
c1 = [-1;-2];
u = 1.0/sqrt(3);
c2 = [0.2;-2];

Fy = 1;
Fx = Fy*u;
F1 = [-Fx; Fy];
Tau1 = cross([c1;0], [F1;0]);
tau1 = Tau1(3);

F2 = [Fx; Fy];
Tau2 = cross([c1;0], [F2;0]);
tau2 = Tau2(3);

W1 = [F1; tau1];
W2 = [F2; tau2];

Tau3 = cross([c2;0], [F1;0]);
tau3 = Tau3(3);

Tau4 = cross([c2;0], [F2;0]);
tau4 = Tau4(3);

W3 = [F1; tau3];
W4 = [F2; tau4];

W = [W1,W2,W3,W4];
W = bsxfun(@rdivide, W, sqrt(sum(W.^2)));
%V = [0.05;0.9;0.05];


numCORs = 200;

radius = 10;
CORs = zeros(numCORs, 2);
V = zeros(numCORs, 3);
flags = zeros(numCORs, 1);
fval = -1 * ones(numCORs,1);
% Sample first half as CCW on the left plane.
CORs(1:numCORs/2, :) = 2 * radius * bsxfun(@minus, rand(numCORs/2, 2), [1,0.5]);
CORs(numCORs/2+1:end, :) = 2 * radius * bsxfun(@minus, rand(numCORs/2, 2), [0,0.5]);
V(1:numCORs/2, :) = GetBodyVelFromCOR(CORs(1:numCORs/2, :), 'CCW');
V(numCORs/2+1:end, :) = GetBodyVelFromCOR(CORs(numCORs/2+1:end, :), 'CW');

ind_perm = randperm(numCORs);
CORs = CORs(ind_perm, :);
V = V(ind_perm, :);

axis equal;

%Draw the polygonal.
hold on;
plot([-2,-2,2,2,0.2,-2], [-2,2,2,0,-2,-2], 'g-');

% Draw the COM. 
plot(0,0, 'ko', 'MarkerSize', 7);
% Draw the friction cone.
plot(c1(1),c1(2), '^', 'MarkerSize', 6); 
plot(c2(1),c2(2), '^', 'MarkerSize', 6); 
s = 1.5;
Fx = Fx * s;
Fy = Fy * s;
plot([c1(1), c1(1)+ Fx], [c1(2), c1(2) + Fy], 'r-');
plot([c1(1), c1(1)- Fx], [c1(2), c1(2) + Fy], 'r-');
plot([c2(1), c2(1)- Fx], [c2(2), c2(2) + Fy], 'r-');
plot([c2(1), c2(1)+ Fx], [c2(2), c2(2) + Fy], 'r-');

for i = 1:1:numCORs
    [f,val] = SearchDiagnolEllipsoid(W, V(i,:)');
    %[f, mu, sigma] = CrossEntropyCheckStable(W, V(i,:)');
    %f = CheckStableDual(W, V(i,:)');
    flags(i) = f;
    fval(i) = val;
    if (f == 0)
        plot(CORs(i,1), CORs(i,2), 'r*');
        fprintf('NotStable %f,%f\n', CORs(i,1), CORs(i,2));
    else
        plot(CORs(i,1), CORs(i,2), 'b*');
        fprintf('Stable %f,%f\n', CORs(i,1), CORs(i,2));
    end
    drawnow;
end

