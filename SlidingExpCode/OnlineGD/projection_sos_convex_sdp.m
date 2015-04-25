function [a, Q] = projection_sos_convex_sdp(c, A, B)
num_LCs = size(B,2);
b = cell(num_LCs);
for i = 1:1:num_LCs
b{i} = B(:,i)';
end
cvx_begin quiet
    variable Q(9,9) semidefinite 
    variable a(15)
    minimize(0.5 * norm(a-c))
    subject to
        for i = 1:1:num_LCs
           trace(A{i} * Q) == b{i} * a;
        end
cvx_end
end

