clc
clear

global dataIndices
addpath('../distgraph/functions')
addpath('../distgraph/shapedist')
initGlobals

datasetIndex = 1;
radius = 4;
numSamples = 100;


datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;
shapeDists = zeros(totalSize,1);
indexPairs = zeros(totalSize,2);

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

vidx = 0;
for i=1:100
    ri = randi(sampleSize);
    ci = randi(sampleSize);
    sampleDiffAbs = abs(samples(ri,:) - samples(ci,:));
    prcvDist = sum(sampleDiffAbs(:, dataIndices.descrs(1)));
    pcwgDist = sum(sampleDiffAbs(:, dataIndices.descrs(2)));
    shdist = 0;
    try
        shdist = shapeDistance(datasetIndex, depthList, radius, samples(ri,:), samples(ci,:), true);
    catch ME
        if strncmpi(ME.identifier, 'shapeDistance', 20)
            warning('%s %s', ME.identifier, ME.message);
        else
            rethrow(ME)
        end
    end
    dists = [shdist prcvDist pcwgDist]
    if shdist < 0.003 && prcvDist < pcwgDist/2
        pause
    end
    vidx = vidx+1;
    shapeDists(vidx) = shdist;
    indexPairs(vidx,:) = [ri ci];
end