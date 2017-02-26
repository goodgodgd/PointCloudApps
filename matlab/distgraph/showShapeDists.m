function showShapeDists()
clear
clc

global radius normalDistWeight dataIndices
addpath('functions')
initGlobals

radius = 5;
dataIndex = 1;
numSamples = 100;
refidx = 20; % 13
pcwgInd = dataIndices.descrs(2);

datasetPath = dataPath(dataIndex, radius)
setCameraParams(datasetPath)
showIndices = [1 5 10 20 50];

% load shape distance
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
shapeDists = shapeDists(:,:,1) + shapeDists(:,:,2)*normalDistWeight;
sdScale = 1000;
shapeDists = shapeDists*sdScale;

% sort distance
[sortShapeDist, sortShapeIndi] = sort(shapeDists, 2);
outShapeDist = sortShapeDist(refidx,showIndices)
outQueryIndi = sortShapeIndi(refidx,showIndices)

% load descriptors
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

% compare two point cloud
for i = 1:length(outShapeDist)
    queidx = outQueryIndi(i)
    shapedist = outShapeDist(i)
    pcModel = loadPCAligned(datasetPath, samplesRefer(refidx,:));
    pcQuery = loadPCAligned(datasetPath, samplesQuery(queidx,:));
    drawPointClouds(pcModel, pcQuery, 0, 1)
    pause
end
end

function drawPointClouds(pcModel, pcQuery, resample, showNormal)
if resample == 1 && pcModel.Count > 100
    modelIndices = randperm(pcModel.Count, 100);
else
    modelIndices = 1:pcModel.Count;
end
if resample == 1 && pcQuery.Count > 100
    queryIndices = randperm(pcQuery.Count, 100);
else
    queryIndices = 1:pcQuery.Count;
end

% plot point clouds
figure(1);
pcshow(pcModel.Location, [0 0 1]);
hold on
pcshow(pcQuery.Location, [1 0 0]);
if showNormal == 1
    quiver3(pcModel.Location(modelIndices,1), pcModel.Location(modelIndices,2), pcModel.Location(modelIndices,3), ...
            pcModel.Normal(modelIndices,1), pcModel.Normal(modelIndices,2), pcModel.Normal(modelIndices,3), 'b');
    quiver3(pcQuery.Location(queryIndices,1), pcQuery.Location(queryIndices,2), pcQuery.Location(queryIndices,3), ...
            pcQuery.Normal(queryIndices,1), pcQuery.Normal(queryIndices,2), pcQuery.Normal(queryIndices,3), 'r');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
% title('compare two shapes')
hold off
descNames = {'Model', 'Query'};
legend(descNames)
end
