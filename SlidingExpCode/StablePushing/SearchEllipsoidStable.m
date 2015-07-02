% Input: wrench cone edges. body velocity.
% Output:
function [flag_stable] = SearchEllipsoidStable(W, V)
options_sdp = sdpoptionset('Algorithm','interior-point',...
                           'MaxIter', 500, ... 
                           'GradConstr','off','GradObj','off','DerivativeCheck','off',...
                           'Aind',1);                % Mark the beginning of the matrix constraints
%                        'L_upp',200,...              % Set upper bounds on the off-diagonal elements of the Cholesky factors    
%                        'L_low',-200);               % Set lower bounds on all elements of the Cholesky factors                     
fval_m = 0;
a_m = zeros(6,1);
N_s = 10;
for i = 1:1:N_s
    %a0 = 0.1*ones(6,1);
    a0 = rand(6,1);
    a0(4) = 0;
    a0(5) = 0;
    a0(6) = 0;
    a0 = a0/sum(a0);
    %aeq = [];
    %beq = [];
    aeq = [0,0,0,1,0,0;
           0,0,0,0,1,0;
           0,0,0,0,0,1;
           1,1,1,1,1,1];
    beq = [0;0;0;1];
    eps_lb = 0.01;
    lb = [eps_lb, eps_lb, eps_lb, 0,0,0];
    [a,fval] = fminsdp(@(x)FunConicRes(x,W,V),a0,[],[],aeq,beq,lb,[],@(x)SemiDefConstraint(x),options_sdp);
    if (fval <= fval_m)
        fval_m = fval;
        a_m = a;
    end
end
    fval = fval_m;
    a = a_m;
    EPS = 1e-6;
    A = [a(1), a(4)/2, a(5)/2;
            a(4)/4, a(2), a(6)/2;
            a(5)/2, a(6)/2, a(3)]
    if (abs(fval) < EPS)
        flag_stable = 1;
        fprintf('Stable:%f,%f,%f\n', abs(fval), min(eig(A)), max(eig(A)));
    else
        flag_stable = 0;

        fprintf('NotStable:%f,%f,%f\n', abs(fval), min(eig(A)), max(eig(A)));
    end   



