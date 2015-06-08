% Input: F and V are column-based example matrix. 3*N.
function [f] = fun(x, F, V, w_force, w_reg)
A = [x(1) x(4)/2 x(5)/2;
    x(4)/2 x(2) x(6)/2;
    x(5)/2 x(6)/2 x(3)];
Vp = A * F;
Vp = bsxfun(@rdivide, Vp, sqrt(sum(Vp.^2,1)));
f = norm(Vp - V, 'fro') + w_force * norm(diag(F' * A * F) - ones(size(F,2), 1)) + w_reg * norm(x);
% for i = 1:1:N
% %     Vp(i,:) = [x(1) * F(i,1) + x(4)/2 * F(i,2) + x(5)/2 * F(i,3);
% %                x(2) * F(i,2) + x(4)/2 * F(i,1) + x(6)/2 * F(i,3);
% %                x(3) * F(i,3) + x(5)/2 * F(i,1) + x(6)/2 * F(i,2)];
%     Err(i) = (Vp(i,:) / norm(Vp(i,:)) - V(i,:)) * (Vp(i,:) / norm(Vp(i,:)) - V(i,:))';
% end
% f = sum(Err);
end

