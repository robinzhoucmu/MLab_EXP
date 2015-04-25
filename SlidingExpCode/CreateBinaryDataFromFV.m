% Input:
% F_{N, 3}: The body forces.
% V_{N, 3}: The body speeds.
% Output:
% xData_{2*N,3}: The training data.
% yData_{2*N,1}: The training label.
function [xData, yData] = CreateBinaryDataFromFV(F,V)
numData = size(F,1)*2 + 1;
numF = size(F,1);
xData = zeros(numData, 3);
yData = zeros(numData, 1);
% Compute the row norm of F.
nF = sqrt(sum(F.^2, 2));

r = 0.05;
xData(1:numF,:) = F + bsxfun(@times, V, nF) * r;
xData(numF + 1:end-1,:) = F - bsxfun(@times, V, nF) * r;
yData(1:numF,:) = 1;
yData(numF + 1:end-1,:) = -1;

% Add the point of origin.
xData(end,:) = [0,0,0];
yData(end,:) = -1;

end

