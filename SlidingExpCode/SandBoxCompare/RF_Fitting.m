function [trees] = RF_Fitting(Dir_Loads_Train, Dir_Vel_Train, Dir_Loads_Test, Dir_Vel_Test)
trees = cell(3,1);
for i = 1:1:3
    trees{i} = TreeBagger(100, Dir_Loads_Train, Dir_Vel_Train(:,i), 'MinLeaf', 5);
end
% Train accuracy.
pred_v_tree = zeros(size(Dir_Vel_Train,1),3);
for i = 1:1:3
    pred_v_tree(:,i) = str2double(trees{i}.predict(Dir_Loads_Train));
end
pred_v_tree = bsxfun(@rdivide, pred_v_tree, sqrt(sum(pred_v_tree.^2,2)));

angles_train = acos(diag(Dir_Vel_Train * pred_v_tree')) * 180 / pi;
disp('Mean Training Angle(Degree) Deviation');
mean(angles_train)

% Test accuracy.
pred_v_tree_test = zeros(size(Dir_Vel_Test,1),3);
for i = 1:1:3
    pred_v_tree_test(:,i) = str2double(trees{i}.predict(Dir_Loads_Test));
end
pred_v_tree_test = bsxfun(@rdivide, pred_v_tree_test, sqrt(sum(pred_v_tree_test.^2,2)));

angles_test = acos(diag(Dir_Vel_Test * pred_v_tree_test')) * 180 / pi;
disp('Mean Test Angle(Degree) Deviation');
mean(angles_test)

end

