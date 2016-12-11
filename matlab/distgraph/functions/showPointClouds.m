function showPointsClouds(datasetIndex, radius, numSamples)

showAllPointClouds(datasetIndex, radius)
% showSampleClouds(datasetIndex, radius, numSamples)

end

function showAllPointClouds(datasetIndex, radius)
descDataPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', descDataPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

datasetPath = workingDir(datasetIndex, radius);
frameCount = 1;

while(1)
    frameCount=frameCount+1;
    filename = sprintf('%s/DDS_%05d.txt', datasetPath, frameCount);
    if(exist(filename, 'file')==0)
        break;
    end
    
    curdata = load(filename);
    % set frame index
    curdata(:,1) = frameCount*ones(size(curdata,1),1);
    
    for idx=1:size(curdata,1)
        index = [frameCount, idx]
        ptcloud = loadPCAligned(descDataPath, depthList, curdata(idx,:), radius);
        drawPointCloud(ptcloud);
        pause
    end
end
pause
end

function showSampleClouds(datasetIndex, radius, numSamples)
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

for idx = 1:size(samples,1)
    idx
    ptcloud = loadPCAligned(datasetPath, depthList, samples(idx,:), radius);
    drawPointCloud(ptcloud);
    pause
end
end

function drawPointCloud(ptcloud)
if ptcloud.Count > 100
    modelIndices = randperm(ptcloud.Count, 100);
else
    modelIndices = 1:ptcloud.Count;
end
% plot point clouds
figure(1);
pcshow(ptcloud.Location, [1 0 0]);
hold on
quiver3(ptcloud.Location(modelIndices,1), ptcloud.Location(modelIndices,2), ptcloud.Location(modelIndices,3), ...
        ptcloud.Normal(modelIndices,1), ptcloud.Normal(modelIndices,2), ptcloud.Normal(modelIndices,3), 'r');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off
end
