N = 500;
theta1 = 0:2*pi/N:2*pi;
theta2 = 0:2*pi/N:2*pi;
w1 = 1;
w2 = 1;
x1 = -1; y1 = 2;
x2 = 2; y2 = 1;
Fx = zeros([N*N, 1]);
Fy = zeros([N*N, 1]);
M = zeros([N*N, 1]);
ind = 0;
for i = 1:1:N
    for j = 1:1:N
        ind = ind + 1;
        Fx(ind) = - w1 * sin(theta1(i)) - w2 * sin(theta2(j));
        Fy(ind) = w1 * cos(theta1(i)) + w2 * cos(theta2(j));
        M(ind) = w1 * (x1 * cos(theta1(i)) + y1 * sin(theta1(i))) + ...
                 w2 * (x2 * cos(theta2(j)) + y2 * sin(theta2(j)));
    end
end

figure, plot3(Fx, Fy, M, 'b.', 'Markersize', 2);