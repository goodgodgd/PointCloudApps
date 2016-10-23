clc
clear

addpath('functions')
addpath('shapedist')
initGlobals

datasetIndex = 1;
radius = 4;
frameIndex = 12;

global dataIndices
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/DDS_%05d.txt', datasetPath, frameIndex);
samples = load(filename);
samples(:,1) = frameIndex+1;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;
distances = struct('shape', zeros(totalSize,1), 'descr', zeros(totalSize,6));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
% depthName = sprintf('%s/%s', datasetPath, depthList{11,1})
% pointCloud = loadPointCloud(depthName);
% return

vidx = 0;
for i=1:100
    vidx = vidx+1;
    ri = randi(sampleSize-1)+1;
    ci = randi(sampleSize-1)+1;
    distances.shape(vidx) = shapeDistance(datasetIndex, depthList, radius ...
                                          , samples(ri,:), samples(ci,:));
    k = input('haha');
end
