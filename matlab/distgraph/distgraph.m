clear
clc

global radius dataIndices
addpath('functions')
initGlobals

numSamples = 100;
radii = [4, 5, 6];
numDsets = dataPath();
numRadii = length(radii);
dsetIndices = 1:numDsets;
top1_accuracy = zeros(numRadii*numDsets,9);
top5_accuracy = zeros(numRadii*numDsets,9);
optGradWeight = zeros(numRadii, numDsets);

% radius = 5;
for radius = radii
    for dataIndex = dsetIndices
        config = [dataIndex radius]
        datasetPath = dataPath(dataIndex, radius)
        setCameraParams(datasetPath)
        
%         D1collectData(datasetPath)
%         D2samplePoints(datasetPath, numSamples)
        D3calcShapeDists(datasetPath, numSamples)
        D4calcDescDists(datasetPath, numSamples);
%         [top1, top5] = D5analyzeResult(datasetPath, numSamples)
%         
%         top1_accuracy((radius-4)*numDsets+dataIndex,:) = [config, top1];
%         top5_accuracy((radius-4)*numDsets+dataIndex,:) = [config, top5];
%         optGradWeight(radius-3, dataIndex) = optimizeGradientWeight(datasetPath, numSamples);
    end
end

return

[~, top1_sorted_indices] = sort(top1_accuracy(:,1:6), 2, 'descend');
top1_pcwg_indices = find(top1_sorted_indices'==2) - 1;
top1_pcwg_raking = mod(top1_pcwg_indices, 6) + 1;
top1_accuracy = [top1_accuracy top1_pcwg_raking]

[~, top5_sorted_indices] = sort(top5_accuracy(:,1:6), 2, 'descend');
top5_pcwg_indices = find(top5_sorted_indices'==2) - 1;
top5_pcwg_raking = mod(top5_pcwg_indices, 6) + 1;
top5_accuracy = [top5_accuracy top5_pcwg_raking]

optGradWeight
