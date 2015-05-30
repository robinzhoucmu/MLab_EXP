% Fit elipsoid (e.g., min volumn enclosing elipsoid) 
% given force and velocities using CVX.
% F, V: 3*N matrix. 
% lambda: tradeoff parameter for fitting force.
% gamma: tradeoff parameter for regularization.
% A: x^Ax - 1 = 0;
function [A, xi, delta, pred_V_dir, s] = FitElipsoidForceVelocityCVX(F, V, lambda, gamma)

[d, n] = size(F);
cvx_begin quiet
    variable A(d,d) semidefinite
    variables xi(n) s(n) delta(n)
minimize( gamma * norm(A, 'fro') + lambda * sum(xi) + sum(delta))
subject to
    for i = 1:n
       norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
       norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
       s(i) >= 1
    end
    %A - 0.1 * eye(3) >= 0
cvx_end

pred_V = A*F;
pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2)));
err = mean(sqrt(sum((pred_V_dir - V).^2)))

end