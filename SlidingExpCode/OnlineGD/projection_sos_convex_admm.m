function [a, Q, lambda] = projection_sos_convex_admm(c, Qt, lambda_prev, A, B)
pho = 10;
iter = 0;
max_iter = 100;
% Initialize ADMM.
a = c;
Q = Qt;
num_LCs = size(B, 2);
lambda = lambda_prev;
%lambda = zeros(num_LCs, 1);
lambda_last = lambda;
while (iter < max_iter)
%a = lag_cvx_a(Q, lambda, c, A, B, pho);
%Q = lag_cvx_Q(a, lambda, A, B, pho);
[a, Q] = lag_cvx_a_Q(Q, a, lambda, c, A, B, pho);
lambda = update_lambda(Q, a, lambda, A, B, pho);
if (norm(lambda - lambda_last) < (pho * 1e-3))
    break;
else
    lambda_last = lambda;
end
iter = iter + 1;
end
iter
end

function [a, Q] = lag_cvx_a_Q(Q, a, lambda, c, A, B, pho)
num_LCs = size(B,2);
cvx_begin quiet
    variable Q(9,9) semidefinite 
    variables z(num_LCs) a(15)
    %minimize(0.5 * norm(a-c) + lambda' * (z - B' * a) + 0.5 * pho * norm(z - B'*a).^2)
    minimize(0.5 * norm(a-c) + lambda' * (z - B' * a) + 0.5 * pho * (z - B'*a)' * (z - B'*a))
    subject to
        for i = 1:1:num_LCs
            z(i) == trace(A{i} * Q);
        end
cvx_end
end    


function [a] = lag_cvx_a(Q, lambda, c, A, B, pho)
num_LCs = size(B,2);
vec_AQ = zeros(num_LCs, 1);
for i = 1:1:num_LCs
        vec_AQ(i) = trace(A{i} * Q);
end
cvx_begin quiet
    variable a(15)
    minimize(0.5 * norm(a-c) + lambda' * (vec_AQ - B' * a) + 0.5*pho*norm(vec_AQ - B'*a))
cvx_end
end

function [Q] = lag_cvx_Q(a, lambda, A, B, pho)
num_LCs = size(B,2);
cvx_begin quiet
    variable Q(9,9) semidefinite
    variable z(num_LCs)
    minimize(lambda' * z + 0.5*pho*norm(z - B'*a))
    subject to
        for i = 1:1:num_LCs
            z(i) == trace(A{i} * Q);
        end
cvx_end
end

function [lambda] = update_lambda(Q, a, lambda_prev, A, B, pho)
num_LCs = size(B,2);
vec_AQ = zeros(num_LCs, 1);
for i = 1:1:num_LCs
        vec_AQ(i) = trace(A{i} * Q);
end

lambda = lambda_prev + pho * (vec_AQ - B' * a);
end