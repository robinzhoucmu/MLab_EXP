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

x = x(1:5:end);
y = y(1:5:end);
angles = angles(1:5:end);

numPos = size(x,1);

figure; plot(1:1:numPos, x - x(1), 'r-'); hold on; plot(1:1:numPos, y- y(1), 'g-'); plot(1:1:numPos, angles- angles(1), 'b-'); 

% Window filter.
windowSize = 5;
b = (1/windowSize) * ones(1,windowSize);
a = 1;

% Butterworth filter.
%[b,a] = butter(10, 0.3);

% FIR1


x = filter(b,a,x);
x = x(windowSize + 1:end);
y = filter(b,a,y);
y = y(windowSize + 1:end);
angles = filter(b,a,angles);
angles = angles(windowSize + 1:end);


numPos = size(x,1);

% Plot position
plot(1:1:numPos, x - x(1), 'r-'); hold on; plot(1:1:numPos, y- y(1), 'g-'); plot(1:1:numPos, angles- angles(1), 'b-'); 

% Plot force
numF = size(force_exp2, 1);
fx = force_exp2(:,1);
fy = force_exp2(:,2);
figure; plot(1:1:numF, fx, 'r-'); hold on; plot(1:1:numF, fy, 'g-');


vx = x(2:end) - x(1:end-1); vy = y(2:end) - y(1:end-1); vangles = angles(2:end) - angles(1:end -1);


numV = size(vx, 1);
figure; plot(1:1:numV, vx, 'r-'); hold on; plot(1:1:numV, vy, 'g-'); plot(1:1:numV, vangles, 'b-');

vx = filter(b,a,vx);
vy = filter(b,a,vy);
vangles = filter(b,a,vangles);
figure, plot(1:1:numV, vx, 'r-'); hold on; plot(1:1:numV, vy, 'g-'); plot(1:1:numV, vangles, 'b-');

% Acceleration.
ax = x(3:end) - 2*x(2:end-1) + x(1:end - 2);
ay = y(3:end) - 2*y(2:end-1) + y(1:end - 2);
aangles = angles(3:end) - 2 * angles(2:end-1) + angles(1:end - 2);
numA = size(ax, 1);
figure, plot(1:1:numA, ax, 'r-'); hold on; plot(1:1:numA, ay, 'g-'); plot(1:1:numA, aangles, 'b-');

ax = filter(b,a,ax);
ay = filter(b,a,ay);
aangles = filter(b,a,aangles);
figure, plot(1:1:numA, ax, 'r-'); hold on; plot(1:1:numA, ay, 'g-'); plot(1:1:numA, aangles, 'b-');
