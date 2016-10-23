function testdist2()
clc
clear

addpath('functions')
addpath('shapedist')
initGlobals

datasetIndex = 1;
radius = 4;
numSamples = 100;

global dataIndices
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/distances_%d.mat', datasetPath, numSamples);
distances = load(filename);
distances = distances.distances;

condIndices = find(distances.shape>0.01 & distances.descr(:,2)<3)

vidx = 0;
for i=1:length(condIndices)
    ri = distances.sample(condIndices(i),1);
    ci = distances.sample(condIndices(i),2);
    [samples(ri,2:3)*2 samples(ci,2:3)*2]
    shdist = shapeDistance(datasetIndex, depthList, radius, samples(ri,:), samples(ci,:), true);
    if shdist < 0
        continue
    end
    vidx = vidx+1;
    distances.shape(vidx) = shdist;

    for ti=1:6
        distances.descr(vidx,ti) = descDistance(samples(ri,dataIndices.descrs(ti)) ...
                                              , samples(ci,dataIndices.descrs(ti)), ti);
    end

    prcv = samples(ri,dataIndices.descrs(1));
    if distances.shape(vidx)>0.008 && distances.descr(vidx,2)<3
        [samples(ri,dataIndices.descrs(2)); samples(ci,dataIndices.descrs(2))]
        descriptorDist = distances.descr(vidx,:)
        k = input('please enter');
    end

end
end

function distance = descDistance(descr1, descr2, typeindex)
L2types = 4;
if sum(L2types==typeindex) > 0
    distance = norm(descr1 - descr2, 2);
else
    distance = norm(descr1 - descr2, 1);
end
end
