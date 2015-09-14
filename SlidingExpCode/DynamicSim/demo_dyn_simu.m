mass = 1.518;
pho = 0.05;
% Use log_file_name = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
% 10 train 10 validation.
A = [0.0499    0.0045   -0.0033;
     0.0045    0.0456   -0.0033;
    -0.0033   -0.0033    0.2320];
A = [0.0499    0.0045   0;
     0.0045    0.0456    0;
    0   0   0.2320];
poly4_coeffs = [ 0.0012
    0.0015
    0.0038
   -0.0000
    0.0009
    0.0008
   -0.0001
    0.0010
    0.0024
    0.0028
    0.0096
    0.0060
    0.0022
   -0.0036
    0.0061];
shift = [-0.05; -0.05];
vertices = [0,0;
            0.15,0;
            0,0.15;]';
%V0 = [0.5;1;pi];
%V0 = [1;1; pi];
%V0 = [2;0.1;-0.5*pi];
V0 = [0.15;-0.15;2*pi];
% mode = 'quadratic';
% dt = 0.0025;
% [T_record, pos_2d, Time] = DynamicSimulation2(mass, pho, A, V0, Pose0, dt, mode);
y0 = [0;0;0;V0];
tspan = 0:0.0001:2.0;
mode = 'poly4';
options = odeset('Events', @StopEvent);
[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho,poly4_coeffs, mode), tspan, y0, options);
%[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho, A), tspan, y0, options);
pos_2d = pos_2d(1:end-3,:);
[Vel, Force] = computeLocalVelAndForce(pos_2d', pho, poly4_coeffs, mode);
dir_Vel = bsxfun(@rdivide, Vel,sqrt(sum(Vel.^2)));
[h_traj1] = PlotFreeSliding(pos_2d', Time, vertices, shift);
h_poly = Plot4thPoly(poly4_coeffs, Force');
hold on;
plot3(Force(1,:), Force(2,:), Force(3,:) , 'r.');
plot3(Force(1, end), Force(2, end), Force(3,end), 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
%%%%

V0 = [0.5;1;pi];
y0 = [0;0;0;V0];
tspan = 0:0.0001:2.0;
mode = 'poly4';
options = odeset('Events', @StopEvent);
[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho,poly4_coeffs, mode), tspan, y0, options);
%[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho, A), tspan, y0, options);
pos_2d = pos_2d(1:end-3,:);
[Vel, Force] = computeLocalVelAndForce(pos_2d', pho, poly4_coeffs, mode);
dir_Vel = bsxfun(@rdivide, Vel,sqrt(sum(Vel.^2)));
[h_traj2] = PlotFreeSliding(pos_2d', Time, vertices, shift);
figure(h_poly);
hold on;
plot3(Force(1,:), Force(2,:), Force(3,:) , 'b.');
plot3(Force(1, end), Force(2, end), Force(3,end), 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

V0 = [0.25;-0.1;3*pi];
y0 = [0;0;0;V0];
tspan = 0:0.0001:2.0;
mode = 'poly4';
options = odeset('Events', @StopEvent);
[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho,poly4_coeffs, mode), tspan, y0, options);
%[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho, A), tspan, y0, options);
pos_2d = pos_2d(1:end-3,:);
[Vel, Force] = computeLocalVelAndForce(pos_2d', pho, poly4_coeffs, mode);
dir_Vel = bsxfun(@rdivide, Vel,sqrt(sum(Vel.^2)));
[h_traj3] = PlotFreeSliding(pos_2d', Time, vertices, shift);
figure(h_poly);
hold on;
plot3(Force(1,:), Force(2,:), Force(3,:) , 'm.');
plot3(Force(1, end), Force(2, end), Force(3,end), 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
