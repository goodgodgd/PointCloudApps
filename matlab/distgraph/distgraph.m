function distgraph()
clear
clc

addpath('functions')
initGlobals
pause on

radii = [4 5 6];
numDsets = dataPath();
dsetIndices = 1:numDsets;
loopIndices = createLoopIndices(radii, dsetIndices)

% prepare representative descriptors and compute shape distances
% prepareData(loopIndices);
% return;

% evaluate precision-1 and -5
[top1_accuracy, top5_accuracy] = evaluatePerformance(loopIndices);

% rank PCWG and append information on accuracy
top1_accuracy = rankAccuracy(top1_accuracy);
top5_accuracy = rankAccuracy(top5_accuracy);
save('../../../top1_accuracy.mat', 'top1_accuracy')
save('../../../top5_accuracy.mat', 'top5_accuracy')
end

function loopIndices = createLoopIndices(radii, dsetIndices)
loopIndices = zeros(length(radii)*length(dsetIndices), 3);
loopcnt = 0;
for radius = radii
    for dataIndex = dsetIndices
        loopcnt = loopcnt+1;
        loopIndices(loopcnt,:) = [loopcnt dataIndex radius];
    end
end
end

function prepareData(loopIndices)
global numSamples radius
loopLength = size(loopIndices,1);

for li = 1:loopLength
    dataIndex = loopIndices(li,2);
    radius = loopIndices(li,3);
    datasetPath = dataPath(dataIndex, radius)
    setCameraParams(datasetPath)

    D1collectData(datasetPath)
    D2samplePoints(datasetPath, numSamples)
    D3calcShapeDists(datasetPath, numSamples)
    ComputeBRANDs(datasetPath, numSamples);
end
end

function [top1_accuracy, top5_accuracy] = evaluatePerformance(loopIndices)
global numSamples radius
loopLength = size(loopIndices,1);
top1_accuracy = zeros(loopLength,10);
top5_accuracy = zeros(loopLength,10);

for li = 1:loopLength
    dataIndex = loopIndices(li,2);
    radius = loopIndices(li,3);
    datasetPath = dataPath(dataIndex, radius)
    setCameraParams(datasetPath)

    D4calcDescDists(datasetPath, numSamples);
    [top1, top5] = D5analyzeResult(datasetPath, numSamples)

    top1_accuracy(li,:) = [dataIndex, radius, top1];
    top5_accuracy(li,:) = [dataIndex, radius, top5];
%     optGradWeight(radius-3, dataIndex) = optimizeGradientWeight(datasetPath, numSamples);
end
end

function accuracy = rankAccuracy(accuracy)
descCols = 3:9;
pcwgIndex = 2;
numDescTypes = length(descCols);
[~, sorted_indices] = sort(accuracy(:,descCols), 2, 'descend');
pcwg_indices = find(sorted_indices'==pcwgIndex) - 1;
pcwg_raking = mod(pcwg_indices, numDescTypes) + 1;
accuracy = [accuracy pcwg_raking]
mean_accuracy = mean(accuracy(:,descCols))
end
