function dstData = filterFineSurface(srcData, datasetPath)
numSrcData = size(srcData,1);
validIndices = 1:numSrcData;
sprintf('filterFineSurface: srcData=%d', numSrcData)

for di=1:numSrcData
    try
        ptcloud = loadPCAligned(datasetPath, srcData(di,:));
        roughness = estimateRoughness(ptcloud);
        acceptableRoughness = 0.0038;
        if ~isempty(strfind(datasetPath, '_gn3'))
            acceptableRoughness = acceptableRoughness*1.1;
        end
        
        if mod(di,1000)==0
            sprintf('filterFineSurface... di=%d, roughness=%f / %f', di, roughness, acceptableRoughness)
        end
        
        if roughness > acceptableRoughness
            validIndices(di) = 0;
%             sprintf('skip drawing rough (invalid) point cloud: frame %d, roughness=%f', ...
%                 srcData(di,1), roughness)
%             drawPointCloud(ptcloud);
%             pause(5)
        end
    catch ME
        ME.identifier
        if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
%             warning('%s %s', ME.identifier, ME.message);
            validIndices(di) = 0;
        else
            rethrow(ME)
        end
    end
end

dstData = srcData(validIndices>0,:);
numDstData = size(dstData,1);
sprintf('filterFineSurface: src=%d to dst=%d', numSrcData, numDstData)
assert(numDstData > 5000)
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
title('irregular surfaces')
hold off
end

function roughness = estimateRoughness(ptcloud)
global radius
estiRadius = radius/2.5/100;
outerRadius = 0.08;
normalTerm = 1/outerRadius;
roughness = zeros(ptcloud.Count,1);
for pi=1:2:ptcloud.Count
    [neibIndices, neibDists] = findNearestNeighbors(ptcloud, ptcloud.Location(pi,:), 10);
    ctNormal = ptcloud.Normal(pi,:);
    ctPoint = ptcloud.Location(pi,:);
    if length(neibIndices) < 2 || norm(ctNormal) < 0.1
        roughness(pi) = 0.01;
        continue
    end
    
    neibIndices = neibIndices(neibDists < estiRadius);
    neibDists = neibDists(neibDists < estiRadius);
    neibIndices = neibIndices(2:end);
    neibDists = neibDists(2:end);

%     if length(neibIndices) > 5
%         neibIndices = neibIndices(1:5);
%         neibDists = neibDists(1:5);

    diff = ptcloud.Location(neibIndices,:) - repmat(ctPoint, length(neibIndices), 1);
    dist = abs(diff*ctNormal');
    outside = neibDists > outerRadius;
    dist(outside) = dist(outside)./(neibDists(outside)*normalTerm);
    roughness(pi) = mean(dist);
end
roughness = roughness(roughness>0);
roughness = mean(roughness);
end

