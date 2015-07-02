function [A,a] = SamplePsdMatrix(mu, sigma)
% a1x^2 + a2y^2 + a3z^2 + a4xy + a5xz + a6yz
EPS = 1e-1;
a = mvnrnd(mu, sigma);
a(4) = 0;
a(5) = 0;
a(6) = 0;
A = [a(1), a(4)/2, a(5)/2;
    a(4)/4, a(2), a(6)/2;
    a(5)/2, a(6)/2, a(3)];
[V, D] = eig(A);
A = V * diag(max(diag(D),EPS)) * V';
A = A / sum(sum(A));
% if (min_eig <=0 )
%     A = A + EPS * eye(3,3);
% end
%A = A / norm(A, 'fro');

a(1:3) = diag(A);
a(4) = A(1,2) * 2;
a(5) = A(1,3) * 2;
a(6) = A(2,3) * 2;
end
