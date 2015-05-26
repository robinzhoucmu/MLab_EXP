function [mu, sigma] = CrossEntropyCheckStable(Fc, V)
mu = zeros(6,1);
sigma = eye(6,6);
N = 1;
Ne = 1;
maxIters = 1;
t = 0;
D = zeros(N, 1);
X = zeros(N, 6);
while (t < maxIters)
    for i = 1:1:N
       [A,a] = SamplePsdMatrix(mu, sigma)
       [D(i),k] = CheckStableCVX(Fc, V, A)
       X(i,:) = a; 
       if (D(i) > 1e-3)
         a
       end
     end
    [dummy, ind] = sort(D, 'descend');
    dummy(1)
    X(ind(1),:)
    mu = mean(X(ind(1:Ne), :))
    sigma = cov(X(ind(1:Ne), :))
    t = t+1;
end

end

