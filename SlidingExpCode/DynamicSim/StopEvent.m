function [value, isterminal, direction] = StopEvent(t, y)
norm(y(4:end))
value = (norm(y(4:end)) > 1e-3) + 0;
isterminal = 1;
direction = 0;
end

