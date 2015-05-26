% Given Force Cone (3*nF) and a particular pushing velocity,
% check for if it is stable or not. 
function [dist, k] = CheckStableCVX(Fc, V, A)

nF = size(Fc, 2);
d = size(Fc, 1);
cvx_begin quiet
    variable k(nF,1) 
    %variable A(d,d) semidefinite
    %minimize(sum(max(0, -k)))
    minimize(norm(V - A * (Fc * k))) 
    subject to 
        k >= 0
cvx_end
dist = norm(V - A*(Fc*k));
end

