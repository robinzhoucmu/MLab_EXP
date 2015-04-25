function [ output_args ] = constrained_gradient_update(d, l, v, E, eta1, eta2,a,Q,lambda_prev,A, B)
pho = 0.5;
iter = 0;
max_iter = 20;
% Initialize ADMM.
a = c;
Q = Qt;
num_LCs = size(B, 2);
lambda = lambda_prev;
%lambda = zeros(num_LCs, 1);
lambda_last = lambda;
while (iter < max_iter)
[a, Q] = lag_cvx_a_Q(Q, a, lambda, c, A, B, pho);
lambda = update_lambda(Q, a, lambda, A, B, pho);
if (norm(lambda - lambda_last) < 1e-3)
    break;
else
    lambda_last = lambda;
end
iter = iter + 1;
end
iter
end

function 

