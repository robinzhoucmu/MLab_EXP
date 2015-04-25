% Generate data
rng(1);
x = [ 0.55  0.0;
      0.25  0.35
     -0.2   0.2
     -0.25 -0.1
     -0.0  -0.3
      0.4  -0.2 ]';
  
v = bsxfun(@times, x, 1./sqrt(sum(x.^2)));

[n,m] = size(x);
eps = 1e-1;
x = x + eps * randn([n,m]);
% Create and solve the model
lambda = 10;
gamma = 2;
cvx_begin
    variable A(n,n) symmetric
    variable b(n)
    variables xi(m) s(m) delta(m) 
%maximize( det_rootn( A )  - lambda1 * norm(xi))
%minimize( lambda * norm(xi) + gamma * sum(delta))
minimize( - det_rootn( A ) + lambda * sum(xi) + gamma * sum(delta))
    subject to
    for i = 1:m
        norm(x(:,i)' * A * x(:,i) -1) <= xi(i) 
        norm(A * x(:,i) - s(i) * v(:,i)) <= delta(i)
        s(i) > 0
    end
cvx_end

% Plot the results
clf
% noangles = 200;
% angles   = linspace( 0, 2 * pi, noangles );
% ellipse  = A \ [ cos(angles) - b(1) ; sin(angles) - b(2) ];
% plot( x(1,:), x(2,:), 'ro', ellipse(1,:), ellipse(2,:), 'b-' );
%axis off
plot( x(1,:), x(2,:), 'ro');
z = sym('z',[2 1]); assume(z,'real'); f = z'*A*z;
hold on;
h = ezplot(f == 1);
set(h, 'Color', 'b');



