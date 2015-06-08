A = [0.6490    0.8503    0.6828
    0.8503    1.8195    1.8085
    0.6828    1.8085    2.1409];

F =[0.9751    0.1167    0.1886
    0.6677    0.2053    0.7155
    0.1144    0.4928    0.8626
    0.6480    0.6793    0.3444
    0.4504    0.6872    0.5700];

V = [0.4155    0.6673    0.6181
    0.3197    0.6517    0.6878
    0.2738    0.6461    0.7124
    0.3404    0.6651    0.6647
    0.3128    0.6583    0.6846];

%options = sdpoptionset('Aind', 1);
options = sdpoptionset('Algorithm','interior-point',...
				       'GradConstr','off','GradObj','off','DerivativeCheck','off',...
                       'Aind',1,...                 % Mark the beginning of the matrix constraints
                       'L_upp',200,...              % Set upper bounds on the off-diagonal elements of the Cholesky factors    
                       'L_low',-200);               % Set lower bounds on all elements of the Cholesky factors                     
aeq = ones(size(x))';
beq = 1;
[x,fval] = fminsdp(@(x)fun(x,F,V),x0,[],[],aeq,beq,[],[],@(x)nonlcon(x),options)