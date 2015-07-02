function [fval] = FunConicRes(x, W, V)
A = [x(1) x(4)/2 x(5)/2;
    x(4)/2 x(2) x(6)/2;
    x(5)/2 x(6)/2 x(3)];
%A = A / sum(sum(A));
[fval] = CheckStableLSNonNeg(W, V, A);
fval = -fval;
end

