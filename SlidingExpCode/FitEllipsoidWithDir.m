% Input:
% F_dir, V_dir: 3*N matrix of force and velocity directions.
% Output:
% Matrix coefficients.
function [A, pred_V_dir] = FitEllipsoidWithDir(F_dir, V_dir)
[d, n] = size(F_dir);
cvx_begin 
    variable A(d,d) semidefinite
maximize(trace((A*F_dir)'* V_dir))
subject to
%10 * eye(3) - A >= 0
%norm(A,2) <= 10 
%trace(A) <= 20
norm(A, 'fro') <= 10
cvx_end

pred_V = A*F_dir;
pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2)));
err = mean(sqrt(sum((pred_V_dir - V_dir).^2)))
end

