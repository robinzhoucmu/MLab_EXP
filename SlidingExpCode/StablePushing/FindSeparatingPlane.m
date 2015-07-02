function [ flag_exist, A ] = FindSeparatingPlane( W, V, y )
cvx_begin quiet
    %variable A(3,3) semidefinite %diagonal
    variable A(3,3) semidefinite diagonal
    subject to
    for i = 1:1:4
        trace(A*(W(:,i)*y')) >= 0
    end
    %A(1,1) == A(2,2)
    %trace(A*diag([1,-1,0])) == 0
    %A(1,1) == A(2,2)
    A(1,1) == 1
    A(2,2) == 1
    trace(A*diag([0,0,1])) >= 0.1
    %A(3,3) >= 0.1
    %A(3,3) <= 10
    %ones(1,3) * A * ones(3,1) == 1
    %A - 0.1*eye(3,3) >= 0
    %A - 10*eye(3,3) <= 0
cvx_end
%cvx_optval
if cvx_optval == Inf || cvx_optval == -Inf
    flag_exist = 0;
else 
    flag_exist = 1;
end

end

