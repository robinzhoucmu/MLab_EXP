function [V] = GetBodyVelFromCOR(CORs, RSense)
numCORs = size(CORs, 1);
V = zeros(numCORs, 3);
x = CORs(:,1);
y = CORs(:,2);
z = x.^2 + y.^2 + 1;
V(:,1) = y ./ z;
V(:,2) = -x ./ z;
V(:,3) = 1 ./ z;

if (strcmp(RSense, 'CW'))
    V = -V;
end

end

