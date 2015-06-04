% Input:
% dir_F, bv: row examples.
% Output:
% hyp: hyper parameters for poly kernel.
function [hyp, err_angle_train, err_angle_test] = GP_Fitting(dir_F_train, bv_train, dir_F_test, bv_test)
covfunc  = {@covPoly, 4}; c = 1; sf =0.1; hyp.cov = log([c;sf]);   % 4th order poly
meanfunc = {'meanZero'};
likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);

iters = 10;
v_gp_train = zeros(size(bv_train));
v_gp_test = zeros(size(bv_test));
for i = 1:1:3
    hyp = minimize(hyp, @gp, iters, @infExact, [], covfunc, likfunc, dir_F_train, bv_train(:,i));
    v_gp_test(:,i) = gp(hyp, @infExact, meanfunc, covfunc, likfunc, dir_F_train, bv_train(:,i), dir_F_test);
    v_gp_train(:,i) = gp(hyp, @infExact, meanfunc, covfunc, likfunc, dir_F_train, bv_train(:,i), dir_F_train);
end

%Normalize v.
v_gp_test = bsxfun(@rdivide, v_gp_test, sqrt(sum(v_gp_test.^2,2)));
angles_test = acos(diag(bv_test * v_gp_test')) * 180 / pi;
err_angle_test = mean(angles_test);

v_gp_train = bsxfun(@rdivide, v_gp_train, sqrt(sum(v_gp_train.^2,2)));
angles_train = acos(diag(bv_train * v_gp_train')) * 180 / pi;
err_angle_train = mean(angles_train);

end

