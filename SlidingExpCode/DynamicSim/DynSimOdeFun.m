% Input:
% y: 1-3: x,y,theta; 4-6: vx, vy, omega;  In global frame.
% m: mass
% pho: radius of gyration.
% coeffs: Linear(Matrix A). 
function [dy] = DynSimOdeFun(t, y, m, pho, coeffs, mode)
if (nargin < 6)
    mode = 'quadratic';
end
dy = zeros(6,1);
% velocity def.
% dy(1) = y(4);
% dy(2) = y(5);
% dy(3) = y(6);
dy(1:3) = y(4:6);
% represent local velocity.
v = zeros(3,1);
theta = y(3);
R = [cos(theta), -sin(theta);
        sin(theta), cos(theta)];
v(1:2) = R' * y(4:5);
v(3) = y(6);
% angular velocity normalization.
v(3) = v(3) * pho;
% Get force.
F = FindForceGivenVel(v, coeffs, mode);
% Compute Acc in local frame.
acc = - F / m;
% un-normalize angular part.
acc(3) = acc(3) / pho;
% Represent acc in global frame.
acc(1:2) = R * acc(1:2);
% acc def.
dy(4:6) = acc;
%y
% dy(4) = acc(1);
% dy(5) = acc(2);
% dy(6) = acc(3);
end

