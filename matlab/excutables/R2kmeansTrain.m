clc
clear

global dataPath numDescTypes bowFeatDim

addpath('../funcrecog')
initGlobals;
videoIndex = 1;
frameIndex = 3;

descIndices = getDescIndicesWords();
lastIndices = descIndices(numDescTypes);
query = sprintf('%s/OBJ*_%d_%d.txt', dataPath, videoIndex, frameIndex);
files = dir(query);
descriptors = zeros(0,lastIndices(end));

for i=1:length(files)
    fileName = sprintf('%s/%s', dataPath, files(i).name);
    descriptors = [descriptors; load(fileName)];
end

size(descriptors)
numClusters = bowFeatDim;

for i=1:numDescTypes
    [clutIndices, centroids, sumd, dists] ...
        = kmeans(descriptors(:, descIndices(i)), numClusters+10, ...
        'Start', 'cluster', 'EmptyAction', 'singleton');

    clutCounts = histc(clutIndices, 1:max(clutIndices));
    [sortedCounts, sortedIndices] = sort(clutCounts, 'descend');
    sortResult = [sortedCounts, sortedIndices]
    freqIndices = sortedIndices(1:numClusters);
    words = centroids(freqIndices,:);
    size(words)
    outputFile = sprintf('%s/word%d.mat', dataPath, i)
    save(outputFile, 'words');
end
