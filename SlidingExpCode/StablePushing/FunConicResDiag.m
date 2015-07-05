function [ fval ] = FunConicResDiag(x, W, V)
A = [x(1) 0 0;
     0 x(2) 0;
     0 0 x(3)];
%A = A / sum(sum(A));
[fval] = CheckStableLSNonNeg(W, V, A);
fval = -fval;
end

