%Input: 
%input signal xin, and alpha(0-1) for new value.
function [xout] = EMA_Filter(xin, alpha)
N = size(xin, 1);
xout = zeros(size(xin));
xout(1,:) = xin(1,:);
for i = 2:1:N
xout(i,:) = alpha * xin(i,:) + (1 - alpha) * xout(i-1,:);
end
end

