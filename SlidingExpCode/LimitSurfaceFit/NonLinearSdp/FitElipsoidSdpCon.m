%Input, F,V: 3*N.
function [A, a, dev_angle] = FitElipsoidSdpCon( F, V, w_force, w_reg)
if nargin == 2
    w_force = 0;
    w_reg = 0;
elseif nargin == 3
    w_reg = 0;
end
N = size(F, 2);
options_sdp = sdpoptionset('Algorithm','interior-point',...
				       'GradConstr','off','GradObj','off','DerivativeCheck','off',...
                       'Aind',1,...                 % Mark the beginning of the matrix constraints
                       'L_upp',200,...              % Set upper bounds on the off-diagonal elements of the Cholesky factors    
                       'L_low',-200);               % Set lower bounds on all elements of the Cholesky factors                     
a0 = ones(6,1);
aeq = ones(size(a0))';
beq = 1;
[a,fval] = fminsdp(@(x)fun(x,F,V,w_force, w_reg),a0,[],[],aeq,beq,[],[],@(x)nonlcon(x),options_sdp);
V_p = predict_v(a, F);
dev_angle = mean(acos(diag(V_p' * V)) * 180 / pi);
A = [a(1) a(4)/2 a(5)/2;
    a(4)/2 a(2) a(6)/2;
    a(5)/2 a(6)/2 a(3)];
end

