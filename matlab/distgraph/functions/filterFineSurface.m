function dstData = filterFineSurface(srcData, datasetIndex, radius)

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

numData = size(srcData,1);
validIndices = 1:numData;
for di=1:numData
    try
        ptcloud = loadPCAligned(datasetPath, depthList, srcData(di,:), radius);
        roughness = estimateRoughness(ptcloud);
        if roughness > 0.003
            validIndices(di) = 0;
            drawPointCloud(ptcloud);
            pause(5)
        end
    catch ME
        ME.identifier
        if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
            warning('%s %s', ME.identifier, ME.message);
            validIndices(di) = 0;
        else
            rethrow(ME)
        end
    end
end

dstData = srcData(validIndices>0,:);
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

function roughness = estimateRoughness(ptcloud)
estiRadius = 0.015;
roughness = zeros(ptcloud.Count,1);
for pi=1:ptcloud.Count
    [neibIndices, dists] = findNearestNeighbors(ptcloud, ptcloud.Location(pi,:), 5);
    neibIndices = neibIndices(dists < estiRadius);
    neibIndices = neibIndices(2:end);
    ctNormal = ptcloud.Normal(pi,:);
    ctPoint = ptcloud.Location(pi,:);

    if length(neibIndices) > 5
        neibIndices = neibIndices(1:5);
    elseif length(neibIndices) < 2 || norm(ctNormal) < 0.1
        roughness(pi) = 0.01;
        continue
    end
    
    diff = ptcloud.Location(neibIndices,:) - repmat(ctPoint, length(neibIndices), 1);
    dist = abs(diff*ctNormal');
    roughness(pi) = mean(dist);
end
roughness = mean(roughness);
end



























