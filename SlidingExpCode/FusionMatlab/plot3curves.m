% Input: signal, time.
function [h] = plot3curves(x, t)
h = figure;
plot(t, x(:,1), 'r-');
hold on;
plot(t, x(:,2), 'g-');
plot(t, x(:,3), 'b-');
end

