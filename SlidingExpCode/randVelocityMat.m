numTrials = 500;
ct = 0;
avgSmallx = 0;
avgSmally = 0;

for trial = 1:1:numTrials
N = 1000;
% Sample the points.
numP = N;
%numP = N;
numC = N * 1.25;

p = randn(2, numP);
%MatP = repmat(p, [N 1]);
%replicate the x-row. 
MatPx = repmat(p(1,:), [numC 1]);
MatPy = repmat(p(2,:), [numC 1]);

% Sample the CORs.
c = randn(2, numC);

MatCx = repmat(c(1,:)', [1 numP]);
MatCy = repmat(c(2,:)', [1 numP]);

Vx = MatPx - MatCx;
Vy = MatPy - MatCy;

% Compute the velocity inverse norm matrix

NormMat = 1./sqrt(Vx.*Vx + Vy.*Vy);

NVx = NormMat * diag(p(1,:)') - diag(c(1,:)') * NormMat; 
NVy = NormMat * diag(p(2,:)') - diag(c(2,:)') * NormMat;

rNVx = rank(NVx);
rNvy = rank(NVy); 
%detx = det(NVx);
%dety = det(NVy);
%smallx = min(svd(NVx));
%smally = min(svd(NVy));
avgSmallx = avgSmallx + smallx;
avgSmally = avgSmally + smally;

if (rNVx < numP || rNvy < numP)
%    fprintf('%f, %f\n', smallx, smally);
    ct = ct + 1;
end
end
ct / numTrials
avgSmallx / numTrials
avgSmally / numTrials