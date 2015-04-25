% Input: 
% Force, Vel: 3*N matrix.
% eta1: The control parameter for l2 regularization.
% eta2: The control parameter for velocity matching.
% num_pass: Number of passes over data.
% lr_rate: constant scaling learning rate.

% Output:
% a: coefficients for the fitted 4th order homogenious polynomial.
function [a, xi] = online_poly4(Force, Vel, eta1, eta2, num_pass, lr_rate, a_init, Q_init)
if (nargin == 4)
    num_pass = 1;
    lr_rate = 1;
end
if (nargin == 5)
    lr_rate = 1;
end
n = size(Force,2);
x = Force(1,:);
y = Force(2,:);
z = Force(3,:);
D = [x.^4; y.^4; z.^4; ... 
    x.^3.*y; x.^3.*z; y.^3.*x; y.^3.*z; z.^3.*x; z.^3.*y; ...
    x.^2.*y.^2; x.^2.*z.^2; y.^2.*z.^2; ...
    x.^2.*y.*z; y.^2.*x.*z; z.^2.*x.*y;];
max(eig(D*D'))
min(eig(D*D'))
G = [x.^3; x.^2.*y; x.^2.*z; x.*y.^2; x.*y.*z; x.*z.^2; y.^3; y.^2.*z; y.*z.^2; z.^3];
[E, A, B] = get_poly4_parameters();
%BTB_inv = inv(B'*B);
% Number of terms in homogenious poly4.
dim_poly4 = 15;
% Initialize paramters.
if (nargin < 7)
    a = ones(dim_poly4, 1) * 0.001;
else
    a = a_init;
end
if (nargin < 8)
    Q = zeros(size(A{1}));
else
    Q = Q_init;
end
lambda = zeros(size(B,2), 1);
ct_pass = 0;
while (ct_pass < num_pass)
    ct_pass
    g_tot = zeros(size(a));
    for i = 1:1:n
        d = D(:,i);
        l = G(:,i);
        v = Vel(:,i);
        [g] = compute_gradient(a, d, l, v, E, eta1, eta2);
        g_tot = g_tot + g;
        step_size = 1.0 / (i + ct_pass * n);
        c = a - lr_rate * step_size * g;
        %[a, Q] = projection_sos_convex(c, Q, A, B);
         norm_g = norm(g)
         %[a, Q, lambda] = projection_sos_convex_admm(c, Q,lambda,  A, B);
        [a, Q] = projection_sos_convex_sdp(c, A, B);
         %a
    end
    %batch.
%      norm_g = norm(g_tot)
%      c = a - lr_rate * 1.0/(ct_pass + 1) * g_tot;
%     [a, Q, lambda] = projection_sos_convex_admm(c, Q,lambda,  A, B);
%      a
    ct_pass = ct_pass + 1;
end
xi = abs(D' * a - ones(size(Force,2), 1));
Plot4thPoly(a, Force');
end

