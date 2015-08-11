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
    w_force = [0.25, 0.5, 1, 2, 4, 8];
end

best_err = 1e+9;
best_dev_angle = 1e+9;
best_w_force = -1;
best_w_reg = -1;
train_err_record = 1e+9;
train_dev_angle_record = 1e+9;

for ind_f = 1:length(w_force)
    for ind_r = 1:length(w_reg)
       if (strcmp(method, 'poly4'))
            [coeffs, xi, delta, pred_v_train, s] = ...
               Fit4thOrderPolyCVX(F_train', V_train', w_reg(ind_r), w_vel, w_force(ind_f), flag_convex, flag_plot);
           % Evaluate on validation set.
           [err ,dev_angle] = EvaluatePoly4Predictor(F_val, V_val, coeffs);
           [train_err, train_dev_angle] = EvaluatePoly4Predictor(F_train, V_train, coeffs);
       elseif (strcmp(method, 'quadratic'))
           [coeffs, xi_elip, delta_elip, pred_v_lr_train, s_lr] = ...
               FitElipsoidForceVelocityCVX(F_train', V_train',  w_force(ind_f),  w_reg(ind_r), flag_convex, flag_plot);
           [err, dev_angle] = EvaluateLinearPredictor(F_val, V_val, coeffs);
           [train_err, train_dev_angle] = EvaluateLinearPredictor(F_train, V_train, coeffs);
       end
       fprintf('poly4: w_force:%f, w_reg:%f, dev_angle:%f, dev_angle_test:%f\n', w_force(ind_f), w_reg(ind_r), train_dev_angle, dev_angle);
       % Update the best so far.
       if (best_dev_angle > dev_angle) 
            best_err = err;
            best_dev_angle = dev_angle;
            train_err_record = train_err;
            train_dev_angle_record = train_dev_angle;
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
para.train_err_record = train_err_record;
para.train_dev_angle_record = train_dev_angle_record;

end

