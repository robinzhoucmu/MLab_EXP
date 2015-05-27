numTrials = 50;
fails = 0;
for rndTrial = 1:1:numTrials
    N = 500;
    r = 10;
    %Sample N point in space;
    Pts = rand(2, N) * r;
    %bsxfun(@plus, Pts, [2;2]);
    
    numC = N;
    %Sample N CORs in space;
    CORs= rand(2,numC) * r;
    %bsxfun(@plus, CORs, [1;1]);
    
    V = zeros(numC*2,N);
    for i = 1:1:N
        v = bsxfun(@minus, Pts, CORs(:,i));
        t = v(2,:);
        v(2,:) = v(1,:);
        v(1,:) = -t;
        v = v * diag(1./sqrt(sum(v.^2)));
        V(2*i-1:2*i, :) = v;
    end
    if (rank(V(1:2:end,:)) < N || rank(V(2:2:end,:)) < N)
        fails = fails + 1;
        rank(V(1:2:end,:))
        rank(V(2:2:end,:))
    end
    %min(svd(V(1:2:end,:)))
%    rank(V(1:2:end,:))
%    rank(V(2:2:end,:))
end
fails / numTrials

