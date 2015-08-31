% Input: 
% push_actions: parsed action log.
% trajectory_log: parsed and pre-processed trajectory log, contains slider
% velocity information (w.r.t) to its initial body frame.
function [push_vels,  flag_stable_empirical, dev_angles] = ExtractVelAndEmpiricalStablity(push_actions, trajectory_log, H_tf, pho,  unit_scale, eps_stable_angle)
if (nargin < 4)
    pho = 0.05;
end
if (nargin < 5)
    unit_scale = 1000;
end
if (nargin < 6)
    eps_stable_angle = 5;
end
slider_vels = trajectory_log.slider_velocities;   % N*3.
num_pushes = size(push_actions.push_points, 2);
push_vels = zeros(3, num_pushes);
for i = 1:1:num_pushes
    if strcmp(push_actions.type{i},'TwoPointRotation')
        % change COR frame.
        cor_append1 = H_tf \ [push_actions.cors(1:3, i);1];
        cor = cor_append1(1:3);
        cor = cor / unit_scale;
        z = sqrt(cor(1)^2 + cor(2)^2 + 1);
        % CCW.
        if (push_actions.rot_angles(i) > 0)
            push_vels(1, i) =  cor(2) / z;
            push_vels(2, i) = - cor(1) / z;
            push_vels(3, i) = 1 / z;
        else
        % CW.
            push_vels(1, i) =  - cor(2) / z;
            push_vels(2, i) = cor(1) / z;
            push_vels(3, i) = - 1 / z;
        end
    elseif strcmp(push_actions.type{i},'TwoPointTranslation')
            push_vels(:,i) = push_actions.push_vectors(:,i);
    end   
end
% normalize angular component.
push_vels(3,:) = push_vels(3,:) * pho;
push_vels = bsxfun(@rdivide, push_vels, sqrt(sum(push_vels.^2)));
% Compare push_vel and slider_vel and determine stablility.
dev_angles = acos(diag(slider_vels * push_vels)) * 180 / pi;
flag_stable_empirical = dev_angles < eps_stable_angle;
end

