function [y, t_y] = first_order_difference(x, t)
y = bsxfun(@rdivide, x(3:end,:) - x(1:end-2,:), t(3:end) - t(1:end-2));
t_y = t(2:end-1);
end

