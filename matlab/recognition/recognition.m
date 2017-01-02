clc
clear

addpath('functions')
radii = [4 5 6];
numWords = [50 100 200];
instance_accuracies = zeros(3,8,3);
category_accuracies = zeros(3,8,3);
wordindex = 0;

for nwords = numWords
    wordindex = wordindex + 1;
    inst_accuracy = zeros(3,6);
    cate_accuracy = zeros(3,6);
    
    for radius = radii
        config = [radius nwords]
        initGlobalsRecog(radius, nwords)
        R1kmeansTrain(radius);
%         R2saveBowFeats(radius);
        inst_accuracy(radius-3,:) = R3instanceRecog(radius);
        cate_accuracy(radius-3,:) = R4categoryRecog(radius);
    end
    
    [~, inst_sorted_indices] = sort(inst_accuracy, 2, 'descend');
    inst_prcv_indices = find(inst_sorted_indices'==1) - 1;
    inst_prcv_raking = mod(inst_prcv_indices, 6) + 1;
    inst_pcwg_indices = find(inst_sorted_indices'==2) - 1;
    inst_pcwg_raking = mod(inst_pcwg_indices, 6) + 1;
    instance_accuracies(:,:,wordindex) = [inst_accuracy inst_prcv_raking inst_pcwg_raking];

    [~, cate_sorted_indices] = sort(cate_accuracy, 2, 'descend');
    cate_prcv_indices = find(cate_sorted_indices'==1) - 1;
    cate_prcv_raking = mod(cate_prcv_indices, 6) + 1;
    cate_pcwg_indices = find(cate_sorted_indices'==2) - 1;
    cate_pcwg_raking = mod(cate_pcwg_indices, 6) + 1;
    category_accuracies(:,:,wordindex) = [cate_accuracy cate_prcv_raking cate_pcwg_raking];
end

'convert to percentage'
instance_accuracies(:,1:6,:) = instance_accuracies(:,1:6,:)*100
category_accuracies(:,1:6,:) = category_accuracies(:,1:6,:)*100
