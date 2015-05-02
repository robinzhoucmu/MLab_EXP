% Fuse object pose, robot pose and force log.
% robot pose comes at the slowest frequency around 30Hz.
% object pose/mocap comes around 60 -70 Hz.
% Force sensor comes around 100 Hz. 

function [fused_obj_cart, fused_robot_cart, fused_force, t_ref] = ...
    sensorFusionByTime(obj_cart, t_obj, robot_cart, t_robot, force_log, t_force)
t_ref = t_robot;
num_t = size(t_ref, 1);
ind_obj = 1;
ind_force = 1;
fused_obj_cart = zeros(num_t - 2, size(obj_cart,2));
fused_force = zeros(num_t - 2, size(force_log,2));
for i = 2:1:num_t-1
    % Find averages from last_t to t_ref(i-1) to t_ref(i+1)    
    % Average obj_cart.
    ct = 0;
    avg = zeros(1, size(obj_cart, 2));
    while (ind_obj <= size(obj_cart,1) && t_obj(ind_obj) <= t_ref(i+1)) 
        avg = avg + obj_cart(ind_obj, :);
        ct = ct + 1;
        ind_obj = ind_obj + 1;
    end
    avg = avg / ct;
    fused_obj_cart(i-1, :) = avg;
    
    % Average force.
    ct = 0;
    avg = zeros(1, size(force_log, 2));
    while (ind_force <= size(force_log,1) && t_force(ind_force) <= t_ref(i+1)) 
        avg = avg + force_log(ind_force,:);
        ct = ct + 1;
        ind_force = ind_force + 1;
    end
    avg = avg / ct;
    fused_force(i-1, :) = avg;
end

t_ref = t_ref(2:end-1);
fused_robot_cart = robot_cart(2:end-1,:);
end

