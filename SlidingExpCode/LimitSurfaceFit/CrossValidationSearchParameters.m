%Input:
%F, V: N*3 row-wise data matrix.
%method: string indicating specific training methods.
function [para] = CrossValidationSearchParameters(F_train, V_train, F_val, V_val, options)
flag_convex = 1;
flag_plot = 0;
% weight for velocity matching is fixed at 1.
w_vel = 1;
% regularization of parameters w.r.t velocity matching.
w_reg = [0, 1, 10];

method = options.method;
flag_dir = options.flag_dir;

% force matching relative to velocity.
if (flag_dir)
    w_force = [0];
else
    w_force = [0, 0.25, 1, 5, 100];
end

best_err = 1e+9;
best_dev_angle = 1e+9;
best_w_force = -1;
best_w_reg = -1;

for ind_f = 1:length(w_force)
    for ind_r = 1:length(w_reg)
       if (strcmp(method, 'poly4'))
            [coeffs, xi, delta, pred_v_train, s] = ...
               Fit4thOrderPolyCVX(F_train', V_train', w_reg(ind_r), w_vel, w_force(ind_f), flag_convex, flag_plot);
           % Evaluate on validation set.
           [err ,dev_angle] = EvaluatePoly4Predictor(F_val, V_val, coeffs);
       elseif (strcmp(method, 'quadratic'))
           [coeffs, xi_elip, delta_elip, pred_v_lr_train, s_lr] = ...
               FitElipsoidForceVelocityCVX(F_train', V_train',  w_force(ind_f),  w_reg(ind_r), flag_convex, flag_plot);
           [err, dev_angle] = EvaluateLinearPredictor(F_val, V_val, coeffs);
       end
       % Update the best so far.
       if (best_dev_angle > dev_angle) 
            best_err = err;
            best_dev_angle = dev_angle;
            best_w_force = w_force(ind_f);
            best_w_reg = w_reg(ind_r);
            best_coeffs = coeffs;
       end
    end
end
para.coeffs = best_coeffs;
para.w_force = best_w_force;
para.w_reg = best_w_reg;
para.w_vel = w_vel;
para.err = best_err;
para.dev_angle = best_dev_angle;
end

