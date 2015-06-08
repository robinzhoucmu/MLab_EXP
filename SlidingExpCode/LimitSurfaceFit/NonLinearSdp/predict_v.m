% F: 3*N.
% Vp: 3*N.
function [ Vp ] = predict_v(x, F)
A = [x(1) x(4)/2 x(5)/2;
    x(4)/2 x(2) x(6)/2;
    x(5)/2 x(6)/2 x(3)];
Vp = A * F;
Vp = bsxfun(@rdivide, Vp, sqrt(sum(Vp.^2,1)));
end

