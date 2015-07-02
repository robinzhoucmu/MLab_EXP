function [ dist, k ] = CheckStableLSNonNeg(Fc, V, A)
[k, dist] = lsqnonneg(A*Fc, V);
end

