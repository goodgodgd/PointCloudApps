clear
clc

global radius
addpath('functions')
initGlobals

numSamples = 100;
radii = [4, 5, 6];
numDsets = dataPath();
numRadii = length(radii);
dsetIndices = 1:numDsets;
loopLength = numRadii*numDsets;
top1_accuracy = zeros(loopLength,10);
top5_accuracy = zeros(loopLength,10);
optGradWeight = zeros(numRadii, numDsets);

loopIndex = zeros(loopLength,3);
loopcnt = 0;
for radius = radii
    for dataIndex = dsetIndices
        loopcnt = loopcnt+1;
        loopIndex(loopcnt,:) = [loopcnt dataIndex radius];
    end
end
loopIndex

for li = 1:loopLength
    dataIndex = loopIndex(li,2)
    radius = loopIndex(li,3)
    datasetPath = dataPath(dataIndex, radius)
    setCameraParams(datasetPath)

%     D1collectData(datasetPath)
%     D2samplePoints(datasetPath, numSamples)
%     D3calcShapeDists(datasetPath, numSamples)
%     ComputeBRANDs(datasetPath, numSamples);
    D4calcDescDists(datasetPath, numSamples);
    [top1, top5] = D5analyzeResult(datasetPath, numSamples)

    top1_accuracy(li,:) = [dataIndex, radius, top1];
    top5_accuracy(li,:) = [dataIndex, radius, top5];
    optGradWeight(radius-3, dataIndex) = optimizeGradientWeight(datasetPath, numSamples);
end

% return

descCols = 3:9;
pcwgIndx = 2;
numDescTypes = length(descCols);
[~, top1_sorted_indices] = sort(top1_accuracy(:,descCols), 2, 'descend');
top1_pcwg_indices = find(top1_sorted_indices'==pcwgIndx) - 1;
top1_pcwg_raking = mod(top1_pcwg_indices, numDescTypes) + 1;
top1_accuracy = [top1_accuracy top1_pcwg_raking]

[~, top5_sorted_indices] = sort(top5_accuracy(:,descCols), 2, 'descend');
top5_pcwg_indices = find(top5_sorted_indices'==pcwgIndx) - 1;
top5_pcwg_raking = mod(top5_pcwg_indices, numDescTypes) + 1;
top5_accuracy = [top5_accuracy top5_pcwg_raking]

optGradWeight
