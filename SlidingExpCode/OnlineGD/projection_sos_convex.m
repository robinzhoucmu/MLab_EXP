% Input:
% c: gradient updated coefficients which needs to be projected back onto
% SOS-Convex Constraints.
% Qt: Q from last iteration t. 
% A,B: Expressing SOS-Convex matrix constraints: Tr(A_i, Q) = b_k^T*v.
% A is a cell of sparse matrices.
% BTB_inv: (B^TB)^(-1), given as input to speed up computation.
% Output:
% a: projected coefficients.
% Q: corresponding hessian polynomial SOS decomposition matrix.
function [a, Q] = projection_sos_convex(c, Qt, A, B)
num_LCs = size(B, 2);
lambda = zeros(num_LCs, 1);
Q = Qt;
Q_last = Qt;
% vec_AQ(i) = Trace(A{i} * Q);
vec_AQ = zeros(num_LCs, 1);
tau = 0;
lr_rate = 0.1;
sum = zeros(size(c));
norm_gd = 1e+9;
diff_Q = 1e+9;
%B = full(B)
while norm_gd > 0.1 && tau < 100 && diff_Q > 0.01
    for i = 1:1:num_LCs
        vec_AQ(i) = trace(A{i} * Q_last);
    end
    lambda = -pinv(B' * B) * (vec_AQ - B' * c);
    %lambda = -(B'* B) \ vec_AQ + B \ c;
    %lambda = -(B'* B) \ vec_AQ + B \ c;
    %lambda = gd_ascent(vec_AQ, B, c);
    %lambda = cvx_solve_lambda(vec_AQ, B, c);
    gd_Q = zeros(size(Q));
    for i = 1:1:num_LCs
        gd_Q = gd_Q + lambda(i) * A{i};
    end
    norm_gd = norm(gd_Q,'fro');
    step_size = 1.0 / (tau + 1);
    %step_size = 1;
    Q_gd_new = Q_last - lr_rate * step_size * gd_Q;
    Q_last = Q;
    [U, Sigma] = eig(Q_gd_new);
    Q = U * max(Sigma, 0) * U';
    diff_Q = norm(Q - Q_last,'fro')
    tau = tau + 1
    sum = sum + B * lambda;
end
a = c + B * lambda;
%a = c + (1.0 / tau) * sum;
end

function [lambda] = gd_ascent(vec_AQ, B, c)
lambda = zeros(size(B,2),1);    
%lambda_last = lambda;
norm_gd_lambda = 1e+9;
lr_rate = 0.01;
ct = 1;
while (norm_gd_lambda > 0.03) 
    gd = - (B' * B) * lambda + vec_AQ - B'*c;
    norm_gd_lambda = norm(gd)
    step_size = 1.0 / sqrt(ct);
    %step_size = 1.0;
    lambda = lambda + lr_rate* step_size * gd;
    ct = ct + 1;
end
end

function [lambda] = cvx_solve_lambda(vec_AQ, B, c)
eps = 0;
%B = full(B);
num_lcs = size(B, 2);
cvx_begin
    variable lambda(num_lcs)
maximize(-0.5 * lambda' * (B'* B + eps*eye(num_lcs)) * lambda + lambda' * (vec_AQ - B'*c))
cvx_end

end

% function [Q] = cvx_solve_Q(lambda, B, c)
% cvx_begin
%     variable Q(9,9) semidefinite
% minimize 0.5 * ( )
% cvx_end
% end

