function [xout] = window_filter(xin, windowSize)
b = (1/windowSize) * ones(1,windowSize);
a = 1;
xout = filter(b,a,xin);
xout(1:windowSize-1,:) = [];
end

