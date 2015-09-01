% Input:
% stable_pred: record from PredictTwoPointsStable.
% flag_label: ground truth logical array. 
% h: figure handler to draw curve on. 
% Output:
% result: N*2, first column is precision, second column is recall.
function [result] = ComputePRCurve(stable_pred, flag_label)
num_thresholds = length(stable_pred.eps_norm);
result = zeros(num_thresholds, 2);
indices_pos = find(flag_label == 1);
indices_neg = find(flag_label == 0);
for i = 1:1:num_thresholds
    flag_pred = stable_pred.flag_stable_pred{i};
    indices_pred_pos = find(flag_pred == 1);
    indices_pred_neg = find(flag_pred == 0);
    num_tp = length(intersect(indices_pred_pos, indices_pos));
    if (num_tp ==0 && length(indices_pred_pos) == 0)
        precision = 0;
    else
        precision = num_tp / length(indices_pred_pos);
    end
    recall = num_tp / length(indices_pos);
    result(i,1) = precision;
    result(i,2) = recall;
end

end

