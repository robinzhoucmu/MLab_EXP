mass = 1.518;
A = [0.0499    0.0045   -0.0033;
     0.0045    0.0456   -0.0033;
    -0.0033   -0.0033    0.2320];
pho = 0.1;

V0 = [0.5;-0.4;pi];
%V0 = [0.5;-0.4; 0];
%V0 = [0;0;pi];
V0 = [5;5;pi];
Pose0 = eye(3,3);

mode = 'quadratic';
dt = 0.001;
[T_record, pos_2d, Time] = DynamicSimulation(mass, pho, A, V0, Pose0, dt, mode);

shift = [-0.05; -0.05];
vertices = [0,0;
            0.15,0;
            0,0.15;]';
[h] = PlotFreeSliding(pos_2d', Time, vertices, shift);