function [cineq, ceq] = SemiDefConstraint(x)
cineq = [];
%ceq(1) = x'*x - 1;
ceq = svec([x(1) x(4)/2 x(5)/2; x(4)/2 x(2) x(6)/2; x(5)/2 x(6)/2 x(3)]);  
%ceq = [svec([x(1) x(4)/2 x(5)/2; x(4)/2 x(2) x(6)/2; x(5)/2 x(6)/2 x(3)] - 0.1 * eye(3,3))];
end

