% This function takes input two point sets and return the transform T
% such that T*Pm gives Pr.
% Input:
% Pm: Set of points represented in motion capture frame, 3 * N
% Pr: Same set of points represented in the robot base frame, 3 * N
% Output:
% R, t: The rotation and translation that gets Pm represented in the 
% robot frame. 
% See reference: Least-Squares Fitting of Two 3d Point Sets
function [R t] = GetTransform(Pm, Pr)
% Mean subtraction.
avgPm = mean(Pm, 2);
avgPr = mean(Pr, 2);
Pm = bsxfun(@minus, Pm, avgPm);
Pr = bsxfun(@minus, Pr, avgPr);
% Find the rotation matrix.
H = Pm * Pr';
% H = U * S * V'
[U S V] = svd(H);
R = V * U';

d = det(R);
%print(d)

t = avgPr - R * avgPm;
end

