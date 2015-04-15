close all;
N = 20;
Pc = rand(3, N) * 300;
alpha = 180/pi;
beta = 45/pi;
gamma = 90/pi;
R_actual = GenRotMatFromEuler(alpha, beta, gamma);
v_actual = [0; 0; -135];
t_actual = [100; -15; 200];

noise_r = 5;
% Generate different poses.
Pm_base = zeros(3, N);
rand_angles = rand(3,N) * pi;

for i = 1:1:N
    R_pose = GenRotMatFromEuler(rand_angles(1,i), rand_angles(2,i), rand_angles(3,i));
    Pm_base(:,i) = R_pose * v_actual + Pc(:,i);
end
Pm = bsxfun(@plus, R_actual * Pm_base , t_actual);
Pm_noisy = Pm + noise_r * randn(3, N);
vmin = v_actual - [0; 0; 100];
vmax = v_actual + [0; 0; 100];

%[R_est t_est v_est] = CaliOpt(Pc, Pm_noisy, rand_angles, vmin, vmax)
v0 = [0;0;0];
[R_est t_est v_est] = AlterDescent(Pc, Pm_noisy, rand_angles, v0)

R_sol = R_actual'
t_sol = mean(Pm_base - R_sol * Pm,2)
v_sol = v_actual