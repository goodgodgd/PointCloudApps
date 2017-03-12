function [top1_acc, top5_acc] = D5analyzeResult(datasetPath, numSamples)

global normalDistWeight radius
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
shapeDists = shapeDists(:,:,1) + shapeDists(:,:,2)*normalDistWeight;
sdScale = 1000;
shapeDists = shapeDists*sdScale;

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
descrDists = load(filename);
descrDists = descrDists.descrDists;

top1_acc = top1Accuracy(shapeDists, descrDists, radius);
top5_acc = top5Accuracy(shapeDists, descrDists, radius);
end

function accuracy = top1Accuracy(shapeDists, descrDists, radius_cm)
[cshapeValue, cshapeIndex] = min(shapeDists);
withinRadius = cshapeValue<radius_cm;
% withinRadius = cshapeValue>0;
cshapeIndex = cshapeIndex(withinRadius);
accuracy = zeros(1,8);
accuracy(8) = sum(withinRadius);

for i=1:7
    [~, cdescrIndex] = min(descrDists(:,:,i));
    cdescrIndex = cdescrIndex(withinRadius);
    accuracy(i) = mean(cshapeIndex==cdescrIndex);
end
end

function accuracy = top5Accuracy(shapeDists, descrDists, radius_cm)
[cshapeValue, cshapeIndex] = min(shapeDists);
withinRadius = cshapeValue<radius_cm;
% withinRadius = cshapeValue>0;
cshapeIndex = cshapeIndex(withinRadius);
accuracy = zeros(1,8);
accuracy(8) = sum(withinRadius);

for i=1:7
    [sortedDists, sdescrIndices] = sort(descrDists(:,:,i), 1);
    if i==7
        sortedDists = sortedDists(1:8,withinRadius)';
    end
    sdescrIndices = sdescrIndices(1:5, withinRadius);
    top5tf = sum(repmat(cshapeIndex, 5, 1)==sdescrIndices, 1);
    accuracy(i) = mean(top5tf);
end
end









