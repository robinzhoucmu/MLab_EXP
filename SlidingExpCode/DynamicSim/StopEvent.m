function [value, isterminal, direction] = StopEvent(t, y)
value = (norm(y(4:end)) > 1e-4) + 0;
isterminal = 1;
direction = 0;
end

