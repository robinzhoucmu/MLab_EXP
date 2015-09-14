num_per_eval = 10;
rng(1);
log_file_poke1 = '../../FusionMatlab/RandPushExp/SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
log_push_action1 = 'Logs2/wood_30_90_30_30_30_90/push_actions_range4_random.txt';
flag_gt1 = [
0
1
0
0
0
0
0
0
0
1
1
0
1
0
0
0
0
0
0
0];
[multi_eval_results1, acc1] = MultiStableActionEval(log_file_poke1, log_push_action1, flag_gt1, num_per_eval);

log_file_poke2 = '../../FusionMatlab/RandPushExp/SensorLogs/wood_10_90_10_10_30_130/exp_08_18_1100_50.txt';
log_push_action2 = 'Logs2/wood_10_90_10_10_30_130/push_actions_range4_random.txt';
flag_gt2 = [
1
0
0
1
0
1
0
0
0
0
0
0
1
0
0
1
1
0
0
0];
[multi_eval_results2, acc2] = MultiStableActionEval(log_file_poke2, log_push_action2, flag_gt2, num_per_eval);

log_file_poke3 = '../../FusionMatlab/RandPushExp/SensorLogs/wood_patch/exp_08_17_0839_50.txt';
log_push_action3 = 'Logs2/wood_patch/push_actions_range4_random.txt';
flag_gt3 = [
0
0
1
1
1
1
0
0
0
0
0
0
0
0
0
0
1
0
0
0  
];
[multi_eval_results3, acc3] = MultiStableActionEval(log_file_poke3, log_push_action3, flag_gt3, num_per_eval);

for ind_train_ratio = 1:1:3
    indices_eval = (ind_train_ratio - 1) * num_per_eval + 1: ind_train_ratio * num_per_eval;
    for ind_method = 1:1:4
        for i = 1:1:length(indices_eval)
            r1(i) = length(find((multi_eval_results1{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt1) == 1)) / length(find(flag_gt1 == 1));
            r2(i) = length(find((multi_eval_results2{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt2) == 1)) / length(find(flag_gt2 == 1));
            r3(i) = length(find((multi_eval_results3{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt3) == 1)) / length(find(flag_gt3 == 1));
            p1(i) = length(find((multi_eval_results1{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt1) == 1)) / ...
                length(find(multi_eval_results1{indices_eval(i)}.pred_flag(:,ind_method) == 1));
            p2(i) = length(find((multi_eval_results2{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt2) == 1)) / ...
                length(find(multi_eval_results2{indices_eval(i)}.pred_flag(:,ind_method) == 1));
            p3(i) = length(find((multi_eval_results3{indices_eval(i)}.pred_flag(:,ind_method) & flag_gt3) == 1)) / ...
                length(find(multi_eval_results3{indices_eval(i)}.pred_flag(:,ind_method) == 1));
        end
        method_accuracy{ind_train_ratio}(1:num_per_eval*3,ind_method) = [acc1(indices_eval, ind_method);
                                                          acc2(indices_eval, ind_method);
                                                          acc3(indices_eval, ind_method);];
        method_recall{ind_train_ratio}(1:num_per_eval*3, ind_method) = [r1';r2';r3'];
        method_precision{ind_train_ratio}(1:num_per_eval*3, ind_method) = [p1';p2';p3'];
    end
    fprintf('accuracy\n');
    mean(method_accuracy{ind_train_ratio})
    std(method_accuracy{ind_train_ratio})
    fprintf('recall\n');
    mean(method_recall{ind_train_ratio})
    std(method_recall{ind_train_ratio})
    fprintf('precision\n');
    mean(method_precision{ind_train_ratio})
    std(method_precision{ind_train_ratio})
    fprintf('------------\n');
end
