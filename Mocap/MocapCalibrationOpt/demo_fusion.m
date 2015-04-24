pos_exp2 = dlmread('../PythonExp/pos.txt', ' ');
force_exp2 = dlmread('../PythonExp/force.txt', ' ');
trans = pos_exp2(:, 1:3);
q_exp2 = pos_exp2(:,4:end);
N = size(q_exp2, 1);
q_l_exp2 = zeros(N, 4);
for i = 1:1:N
q_l_exp2(i,:) = qMul(q_exp2(i,:)', q_mo_lo);
end

close all;
angles = atan2(q_l_exp2(:,4), q_l_exp2(:,1)) * 2 * 180 / pi;
x = pos_exp2(:,1);
y = pos_exp2(:,2);
% Plot position
figure; plot(1:1:N, x - x(1), 'r-'); hold on; plot(1:1:N, y- y(1), 'g-'); plot(1:1:N, angles- angles(1), 'b-'); 

% Plot force
numF = size(force_exp2, 1);
fx = force_exp2(:,1);
fy = force_exp2(:,2);
figure; plot(1:1:N, fx, 'r-'); hold on; plot(1:1:N, fy, 'g-');

vx = x(2:end) - x(1:end-1); vy = y(2:end) - y(1:end-1); vangles = angles(2:end) - angles(1:end -1);
figure; plot(1:1:N-1, vx, 'r-'); hold on; plot(1:1:N-1, vy, 'g-'); plot(1:1:N-1, vangles, 'b-');