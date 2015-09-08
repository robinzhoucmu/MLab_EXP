result_stable_data = ...
[1	1	1	0	1
1	1	1	1	1
1	1	1	1	1
1	1	0	0	1
1	1	1	1	1
1	1	0	0	0
1	1	1	1	1
1	0	0	0	1
1	1	0	1	1
1	0	1	0	0
1	1	1	0	1
1	1	1	1	1
1	1	1	1	1
1	1	0	0	1
1	1	1	1	1
1	1	0	0	0
1	1	1	1	1
1	0	0	0	1
1	1	0	1	1
1	0	1	0	0
1	1	1	1	1
1	1	1	1	0
1	1	1	1	1
1	1	0	0	1
1	1	1	1	1
1	1	0	0	1
1	1	1	1	1
1	0	1	1	1
1	1	0	0	1
1	1	0	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
];
result_stable_data_rand = ...
[
0	0	0	0	0
1	1	1	1	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
1	1	1	0	1
0	0	1	0	1
0	0	0	0	0
1	1	1	1	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	1	0	0
1	0	0	0	0
0	0	0	0	0
1	1	1	0	1
0	0	0	0	0
0	0	0	0	0
1	1	1	0	1
0	0	0	0	0
1	1	0	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	1	1	0	0
0	0	0	0	0
0	0	0	0	0
1	1	1	0	1
0	0	0	1	0
1	1	1	0	0
1	1	0	0	1
1	1	0	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
1	1	0	0	1
1	1	1	1	1
1	1	0	0	1
1	1	1	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
1	1	1	0	1
0	0	0	0	0
0	0	0	0	0
0	0	0	0	0
];
result_stable_all = [result_stable_data;result_stable_data_rand];
confusion_mat = zeros(4,4);
for ind = 1:1:4
    confusion_mat(ind,1) = length(find(result_stable_all(:,ind) == 1 & result_stable_all(:,5) == 1));
    confusion_mat(ind,2) = length(find(result_stable_all(:,ind) == 1 & result_stable_all(:,5) == 0));
    confusion_mat(ind,3) = length(find(result_stable_all(:,ind) == 0 & result_stable_all(:,5) == 1));
    confusion_mat(ind,4) = length(find(result_stable_all(:,ind) == 0 & result_stable_all(:,5) == 0));
end
accuracy = (confusion_mat(:,2) + confusion_mat(:,3)) ./ (confusion_mat(:,1) + confusion_mat(:,4))


confusion_mat_rand = zeros(4,4);
for ind = 1:1:4
    confusion_mat_rand(ind,1) = length(find(result_stable_data_rand(:,ind) == 1 & result_stable_data_rand(:,5) == 1));
    confusion_mat_rand(ind,2) = length(find(result_stable_data_rand(:,ind) == 1 & result_stable_data_rand(:,5) == 0));
    confusion_mat_rand(ind,3) = length(find(result_stable_data_rand(:,ind) == 0 & result_stable_data_rand(:,5) == 1));
    confusion_mat_rand(ind,4) = length(find(result_stable_data_rand(:,ind) == 0 & result_stable_data_rand(:,5) == 0));
end
accuracy_rand = (confusion_mat_rand(:,1) + confusion_mat_rand(:,4)) / 60;
