% This function return the rotation matrix given z-x-z
% Input (in radian)
    % alpha: rotation about z
    % beta: rotation about x'
    % gamma: rotation about z'
function [R] = GenRotMatFromEuler(alpha, beta, gamma)
    R1 = [cos(alpha) sin(alpha) 0;
          -sin(alpha) cos(alpha) 0;
          0 0 1]; 
    R2 = [1 0 0;
          0 cos(beta) sin(beta);
          0 -sin(beta) cos(beta)];
    R3 = [cos(gamma) sin(gamma) 0;
          -sin(gamma) cos(gamma) 0
          0 0 1];
    R = R3 * R2 * R1;
end
