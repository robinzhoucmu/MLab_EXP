mass = 1.518;
A = [0.0499    0.0045   -0.0033;
     0.0045    0.0456   -0.0033;
    -0.0033   -0.0033    0.2320];
pho = 0.1;

%V0 = [0.5;-0.4;pi];
%V0 = [0.5;-0.4; 0];
%V0 = [0;0;pi];
V0 = [1;1;pi];
% Pose0 = eye(3,3);
% 
% mode = 'quadratic';
% dt = 0.0025;
% [T_record, pos_2d, Time] = DynamicSimulation2(mass, pho, A, V0, Pose0, dt, mode);

y0 = [0;0;0;V0];
tspan = 0:0.005:1;
[Time,pos_2d] = ode45(@(t,y)DynSimOdeFun(t,y,mass,pho,A), tspan, y0);
shift = [-0.05; -0.05];
vertices = [0,0;
            0.15,0;
            0,0.15;]';
[h] = PlotFreeSliding(pos_2d', Time, vertices, shift);