function showShapeDists()
clear
clc

global radius normalDistWeight dataIndices
addpath('functions')
initGlobals

radius = 5;
dataIndex = 1;
numSamples = 100;
refidx = 15; % 15
pcwgInd = dataIndices.descrs(2);

datasetPath = dataPath(dataIndex, radius);
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

pcModel = loadPCAligned(datasetPath, samplesRefer(refidx,:));
pcModel = pointCloud(pcModel.Location*1000, 'Normal', pcModel.Normal);
drawPointCloud(pcModel, 100, 0)
pause

% compare two point cloud
for i = 1:length(outShapeDist)
    queidx = outQueryIndi(i)
    shapedist = outShapeDist(i)
    pcQuery = loadPCAligned(datasetPath, samplesQuery(queidx,:));
    pcQuery = pointCloud(pcQuery.Location*1000, 'Normal', pcQuery.Normal);
    drawPointClouds(pcModel, pcQuery, 100, 0);
    pause
end
end

function drawPointClouds(pcModel, pcQuery, resample, showNormal)
if resample > 0 && pcModel.Count >= resample
    modelIndices = randperm(pcModel.Count, resample);
else
    modelIndices = 1:pcModel.Count;
end
if resample > 0 && pcQuery.Count >= resample
    queryIndices = randperm(pcQuery.Count, resample);
else
    queryIndices = 1:pcQuery.Count;
end

% plot point clouds
figure(1);
pcshow(pcModel.Location(modelIndices,:), [0 0 1]);
hold on
pcshow(pcQuery.Location(queryIndices,:), [1 0 0]);
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
view(0,-90)
end

