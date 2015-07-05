function [ flag_exist, A ] = FindSeparatingPlane( W, V, y )
tau_max = 2*sqrt(2);
tau_min = 0.1;
cvx_begin quiet
    %variable A(3,3) semidefinite %diagonal
    variable A(3,3) semidefinite diagonal
    minimize 1
    subject to
    for i = 1:1:4
        trace(A*(W(:,i)*y')) >= 0
    end
    [1;0;0]' * A * [1;0;0] == 1;
    [0;1;0]' * A * [0;1;0] == 1;
    [0;0;tau_max]' * A * [0;0;tau_max] >= 1
    [0;0;tau_min]' * A * [0;0;tau_min] <= 1
    %A(1,1) == A(2,2)
    %trace(A*diag([1,-1,0])) == 0
    %A(1,1) == A(2,2)
    %A(1,1) == 1
    %A(2,2) == 1
    %trace(A*diag([0,0,1])) >= 0.1
    %A(3,3) >= 0.2
    %A(3,3) <= 10
    %ones(1,3) * A * ones(3,1) == 1
    %A - 0.1*eye(3,3) >= 0
    %A - 10*eye(3,3) <= 0
cvx_end
%cvx_optval
%if cvx_optval == Inf || cvx_optval == -Inf  
if cvx_optval ~= 1
    flag_exist = 0;
else 
    %cvx_optval
    %A
    flag_exist = 1;
end

end

