function R1kmeansTrain(radius)
% clc
% clear
% radius = 4;

global rawDataPath dataPath numDescTypes bowFeatDim
videoIndex = 1;
frameIndex = 103;

descIndices = getDescIndicesWords();
lastIndices = descIndices(numDescTypes);
query = sprintf('%s/OBJ*V%02dF%03d.txt', rawDataPath, videoIndex, frameIndex)
files = dir(query);
descriptors = zeros(0,lastIndices(end));
querySize = size(files)

for i=1:length(files)
    descriptors = [descriptors; loadDescriptors(files(i).name)];
end

if size(descriptors,1) > 10000
    interval = size(descriptors,1)/10000;
    sampleIndices = 1:interval:size(descriptors,1);
    sampleIndices = floor(sampleIndices);
    descriptors = descriptors(sampleIndices,:);
end

descSize = size(descriptors)
numClusters = bowFeatDim;

for i=1:numDescTypes
    [clutIndices, centroids, sumd, dists] ...
        = kmeans(descriptors(:, descIndices(i)), floor(numClusters+1.05), ...
                'Distance', 'cityblock', 'EmptyAction', 'singleton', 'MaxIter', 200);

    clutCounts = histc(clutIndices, 1:max(clutIndices));
    [sortedCounts, sortedIndices] = sort(clutCounts, 'descend');
    sortResult = [sortedCounts, sortedIndices];
    freqIndices = sortedIndices(1:numClusters);
    words = centroids(freqIndices,:);
    size(words)
    outputFile = sprintf('%s/word%d_%d.mat', dataPath, i, bowFeatDim)
    save(outputFile, 'words');
end
