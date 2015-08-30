mass = 1.518;
pho = 0.05;
% Use log_file_name = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
% 10 train 10 validation.
A = [0.0499    0.0045   -0.0033;
     0.0045    0.0456   -0.0033;
    -0.0033   -0.0033    0.2320];

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

%V0 = [0.5;-0.4;pi];
V0 = [1;1; pi];
%V0 = [0;0;pi];
%V0 = [2.5;2.5;pi];
% Pose0 = eye(3,3);
% 
% mode = 'quadratic';
% dt = 0.0025;
% [T_record, pos_2d, Time] = DynamicSimulation2(mass, pho, A, V0, Pose0, dt, mode);

y0 = [0;0;0;V0];
tspan = 0:0.001:0.5;
mode = 'poly4';
options = odeset('Events', @StopEvent);
[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho,poly4_coeffs, mode), tspan, y0, options);
%[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho, A), tspan, y0, options);
shift = [-0.05; -0.05];
vertices = [0,0;
            0.15,0;
            0,0.15;]';
[h] = PlotFreeSliding(pos_2d', Time, vertices, shift);