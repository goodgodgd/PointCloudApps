clc
clear

global totalDimension; global eachDescIndices;

addpath('../utils')
initGlobals;
dsetPath = 'E:/PaperData/rgbd-object-dataset/_descriptor1'
videoIndex = 1;
frameIndex = 3;

query = sprintf('%s/OBJ*_%d_%d.txt', dsetPath, videoIndex, frameIndex);
files = dir(query);
descriptors = zeros(0,totalDimension);

for i=1:length(files)
    fileName = sprintf('%s/%s', dsetPath, files(i).name);
    descriptors = [descriptors; load(fileName)];
end

size(descriptors)
numCluts = 100;

for i=1:eachDescIndices.length()
    [clutIndices, centroids, sumd, dists] ...
        = kmeans(descriptors(:, eachDescIndices(i)), numCluts+10, ...
        'Start', 'cluster', 'EmptyAction', 'singleton');

    clutCounts = histc(clutIndices, 1:max(clutIndices));
    [sortedCounts, sortedIndices] = sort(clutCounts, 'descend');
    sortResult = [sortedCounts, sortedIndices]
    freqIndices = sortedIndices(1:numCluts);
    words = centroids(freqIndices,:);
    size(words)
    outputFile = sprintf('%s/word%d.mat', dsetPath, i)
    save(outputFile, 'words');
end
