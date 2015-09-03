% Script to generate a stable pushing plot.
% 
h = figure;
hold on;
axis equal;
axis([-400 400 -400 400]);

p1 = [-150/sqrt(2), -25*sqrt(2)]; p2 = [150/sqrt(2), -25*sqrt(2)]; p3 = [0, 50*sqrt(2)];
l12 = createLine(p1,p2); 
l23 = createLine(p2,p3); 
l31 = createLine(p3,p1);

% Plot the COM. 
plot([0;0], 'Marker', '.', 'MarkerSize', 8, 'Color', 'r');

% Plot the fingers. 
