% Input: Force, Vel N*3.
function [beta, err, dev_angle] = LinearRegressionFV(Force, Vel)
beta = mvregress(Force, Vel);
[err, dev_angle] = EvaluateLinearPredictor(Force, Vel, beta);
end

