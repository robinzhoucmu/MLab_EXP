c1 = [-1;-1];
u = sqrt(3);
c2 = [1;-1];

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

V = [0;1;0];
[mu, sigma] = CrossEntropyCheckStable(W, V);
