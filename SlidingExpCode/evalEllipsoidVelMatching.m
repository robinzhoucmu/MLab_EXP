function [err, pred_V_dir] = evalEllipsoidVelMatching(A, F, V_dir)
pred_V = A*F;
pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2)));
disp('Vel Matching Error')
err = mean(sqrt(sum((pred_V_dir - V_dir).^2)))
end

