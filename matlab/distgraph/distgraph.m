clc
clear

addpath('functions')
initGlobals
datasets = 1:3;
radii = [4, 5, 6];
numSample = 100;
top1_accuracy = zeros(9,7);
top5_accuracy = zeros(9,7);

for index = datasets
    for radius = radii
        config = [index radius]
%         D1collectData(index, radius);
%         D2samplePoints(index, radius, numSample);
%         D3calcShapeDists(index, radius, numSample);
%         D4calcDescDists(index, radius, numSample);
        [top1, top5] = D5analyzeResult(index, radius, numSample);
        top1_accuracy(index*3+radius-6,:) = top1;
        top5_accuracy(index*3+radius-6,:) = top5;

%         showDistances(index, radius, numSample);
%         showPointClouds(index, radius, numSample);
    end
end

[~, top1_sorted_indices] = sort(top1_accuracy(:,1:6), 2, 'descend');
top1_pcwg_indices = find(top1_sorted_indices'==2) - 1;
top1_pcwg_raking = mod(top1_pcwg_indices, 6) + 1;
top1_prcv_indices = find(top1_sorted_indices'==1) - 1;
top1_prcv_raking = mod(top1_prcv_indices, 6) + 1;
top1_accuracy = [top1_accuracy top1_prcv_raking top1_pcwg_raking]

[~, top5_sorted_indices] = sort(top5_accuracy(:,1:6), 2, 'descend');
top5_pcwg_indices = find(top5_sorted_indices'==2) - 1;
top5_pcwg_raking = mod(top5_pcwg_indices, 6) + 1;
top5_prcv_indices = find(top5_sorted_indices'==1) - 1;
top5_prcv_raking = mod(top5_prcv_indices, 6) + 1;
top5_accuracy = [top5_accuracy top5_prcv_raking top5_pcwg_raking]
