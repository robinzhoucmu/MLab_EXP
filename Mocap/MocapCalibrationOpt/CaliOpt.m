% Shifting vector from tool center to mocap marker, assuming tool center
% has fixed rotational pose, e.g., always pointing downwards.
% Input:
% Pm: Columnar Matrix (3 * N) of Mocap marker points in Mocap frame.
% Pc: Columnar Matrix (3 * N) of robot tool center points in robot base frame. 
function [R t v] = CaliOpt(Pc, Pm, rand_angles, v0, vm)
N = size(Pc,2);
% Alterating procedure.
maxIter = 40;
iter = 0;
diff = vm - v0;
minError = 1e+9;
Pr = zeros(3, N);
while iter < maxIter
    % Marker point in base frame by shifting the tool center by v0. 
    vs = v0 + (iter/maxIter) * diff 
    %Pr = bsxfun(@plus, Pc, vs);
    for i = 1:1:N
        R_pose = GenRotMatFromEuler(rand_angles(1,i), rand_angles(2,i), rand_angles(3,i));
        Pr(:,i) = R_pose * vs + Pc(:,i);
    end
    [Rs ts] = GetTransform(Pm, Pr)
    Q = bsxfun(@plus, Rs * Pm, ts);
    ls_err = norm(Q - Pr, 'fro')
    plot(iter, ls_err, 'r*');
    hold on;
    if ls_err < minError
        minError = ls_err;
        R = Rs;
        t = ts;
        v = vs;
    end
    iter = iter + 1;
end


end