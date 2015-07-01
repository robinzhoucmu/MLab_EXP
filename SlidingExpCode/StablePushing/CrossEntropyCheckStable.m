function [flag_stable, mu, sigma] = CrossEntropyCheckStable(Fc, V)
mu = zeros(6,1);
sigma = eye(6,6);
N = 50;
Ne = 6;
maxIters = 200;
t = 0;
D = zeros(N, 1);
X = zeros(N, 6);
eps_dist = 1e-3;
flag_stable = 1;
eps_var = 1e-4;
while (t < maxIters && flag_stable && trace(sigma) > eps_var)
    for i = 1:1:N
       [A,a] = SamplePsdMatrix(mu, sigma);
       %[D(i),k] = CheckStableCVX(Fc, V, A);
       [D(i),k] = CheckStableLSNonNeg(Fc, V, A);
       X(i,:) = a; 
       if (D(i) > eps_dist)
         fprintf('NotStable:%f,%f,%f\n', D(i), min(eig(A)), max(eig(A)));
         A
         flag_stable = 0;
         break;
         %k
         %a
         %A
       else
         %fprintf('Stable:%f,%f,%f\n', D(i),min(eig(A)), max(eig(A)));
       end
     end
    [dummy, ind] = sort(D, 'descend');
    mu = mean(X(ind(1:Ne), :));
    sigma = cov(X(ind(1:Ne), :));
    t = t+1;
end

end

