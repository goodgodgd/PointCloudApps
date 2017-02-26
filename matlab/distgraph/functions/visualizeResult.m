% function visualizeResult(datasetIndex, radius, numSamples)
function visualizeResult()
clc
clear
datasetIndex=1
radius=4
numSamples=100

'visualizeResult'
global normalDistWeight
datasetPath = workingDir(datasetIndex, radius);

filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
refnum = size(samplesRefer,1)
samplesQuery = samples.samplesQuery;
quenum = size(samplesQuery,1)

filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
shapeDists = shapeDists(:,:,1) + shapeDists(:,:,2)*normalDistWeight;
sdScale = 1000;
shapeDists = shapeDists*sdScale;

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
descrDists = load(filename);
descrDists = descrDists.descrDists;

% for each reference shape
for ri=1:refnum
    % show closest query shape in terms of shape distance
    [min_qval, min_qidx] = min(shapeDists(ri,:));
    pcModel = loadPCAligned(datasetPath, samplesRefer(ri,:));
    pcQuery = loadPCAligned(datasetPath, samplesQuery(min_qidx,:));
    drawPointClouds(pcModel, pcQuery)
    
    % print corresponding descriptor distance
    descriptor_distances = reshape(descrDists(ri,min_qidx,:), 1, [])
    descriptor_min_dists = min(reshape(descrDists(ri,:,:), size(descrDists,2), []))
    
    pause
    
    % for each descriptor type
        % show the closest query shape in terms of descriptor distance
end

end


function drawPointClouds(pcModel, pcQuery)
if pcModel.Count > 100
    modelIndices = randperm(pcModel.Count, 100);
else
    modelIndices = 1:pcModel.Count;
end
if pcQuery.Count > 100
    queryIndices = randperm(pcQuery.Count, 100);
else
    queryIndices = 1:pcQuery.Count;
end

% plot point clouds
figure(1);
pcshow(pcModel.Location, [1 0 0]);
hold on
pcshow(pcQuery.Location, [0 0 1]);
quiver3(pcModel.Location(modelIndices,1), pcModel.Location(modelIndices,2), pcModel.Location(modelIndices,3), ...
        pcModel.Normal(modelIndices,1), pcModel.Normal(modelIndices,2), pcModel.Normal(modelIndices,3), 'r');
quiver3(pcQuery.Location(queryIndices,1), pcQuery.Location(queryIndices,2), pcQuery.Location(queryIndices,3), ...
        pcQuery.Normal(queryIndices,1), pcQuery.Normal(queryIndices,2), pcQuery.Normal(queryIndices,3), 'b');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('compare two shapes')
hold off
descNames = {'Model', 'Query'};
legend(descNames)
end
