% Fit elipsoid (e.g., min volumn enclosing elipsoid) 
% given force and velocities using CVX.
% F, V: 3*N matrix. 
% A: x^Ax - 1 = 0;
function [A] = FitElipsoidForceVelocityCVX(F, V, lambda, gamma)

[d, n] = size(F);
cvx_begin sdp
    variable A(d,d) symmetric
    variables xi(n) s(n) delta(n)
%minimize( -det_rootn(A) + lambda * sum(xi) + gamma * sum(delta))
minimize( norm(A, 'fro') + lambda * sum(xi) + gamma * sum(delta))
subject to
    for i = 1:n
       norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
       norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
       s(i) >= 0
    end
cvx_end
end