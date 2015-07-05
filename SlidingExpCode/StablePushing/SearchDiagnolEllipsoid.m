function [flag_stable] = SearchDiagnolEllipsoid(W, V)
tau_max = 2*sqrt(2);
tau_min = 0.1;
A = [0,0,1;
     0,0,-1];
b = [1/(tau_min)^2; 
     -1/(tau_max)^2];
 
%  1/t_min^2 >= x3 >= 1/t_max^2

A_eq = [1,0,0;
        0,1,0];
b_eq = [1;1];
options = optimoptions(@fmincon, 'Algorithm', 'active-set');
x0 = [1;1;1];
[x, fval] = fmincon(@(x)FunConicResDiag(x, W, V),x0, A, b, A_eq, b_eq, [], [], [], options)
fval_m = abs(fval);

EPS = 1e-5;
% N = 3000;
% ub = 1/(tau_min^2);
% ul = 1/(tau_max^2);
% delta = (ub - ul) / N;
% fval_m = 0;
% x = ones(3,1);
% x_m = zeros(3,1);
% for i = 1:1:N
% x(3) = ul + i * delta;
% f = FunConicResDiag(x, W, V);
% if (abs(f) > fval_m)
%     fval_m = abs(f);
%     x_m = x;
% end
% if (fval_m >= EPS)
%     fval_m
%     x
%     break;
% end
% end


if fval_m < EPS 
    flag_stable = 1;
else
    flag_stable = 0;
end

