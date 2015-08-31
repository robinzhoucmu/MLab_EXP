% Input:
% dir_F, bv: row examples. Assume the symetry augmentation has been done.
% Output:
% hyp: hyper parameters for poly kernel.
function [hyp, err_angle_train, err_angle_test, v_gp_test] = GP_Fitting(dir_F_train, bv_train, dir_F_test, bv_test, prior)
if (nargin < 5)
    sn = 0.5;
    l = 0.1;
else
    sn = prior.sn;
    l = prior.l;
end
flag_has_testV = ~isempty(bv_test);
%covfunc  = {@covPoly, 1}; c = 0; sf =0.1; hyp.cov = log([c;sf]);   % 4th order poly
%covfunc = {@covLIN}; hyp = [];
covfunc = {@covSEisoU}; 
hyp.cov = log(l); 
meanfunc = {'meanZero'};
likfunc = @likGauss; 
hyp.lik = log(sn);

iters = 10;
v_gp_train = zeros(size(bv_train));
num_test = size(dir_F_test, 1);
v_gp_test = zeros(num_test, 3);
for i = 1:1:3
    %hyp = minimize(hyp, @gp, iters, @infExact, [], covfunc, likfunc, dir_F_train, bv_train(:,i));
    v_gp_test(:,i) = gp(hyp, @infExact, meanfunc, covfunc, likfunc, dir_F_train, bv_train(:,i), dir_F_test);
    v_gp_train(:,i) = gp(hyp, @infExact, meanfunc, covfunc, likfunc, dir_F_train, bv_train(:,i), dir_F_train);
end

%Normalize v.
v_gp_test = bsxfun(@rdivide, v_gp_test, sqrt(sum(v_gp_test.^2,2)) + eps);
err_angle_test = [];
if (flag_has_testV)
    angles_test = acos(diag(bv_test * v_gp_test')) * 180 / pi;
    err_angle_test = mean(angles_test);
end
v_gp_train = bsxfun(@rdivide, v_gp_train, sqrt(sum(v_gp_train.^2,2)) + eps);
angles_train = acos(diag(bv_train * v_gp_train')) * 180 / pi;
err_angle_train = mean(angles_train);

end

