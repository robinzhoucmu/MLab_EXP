function [y] = standardize(x)
y = bsxfun(@rdivide, bsxfun(@minus, x, mean(x)), std(x));
end

