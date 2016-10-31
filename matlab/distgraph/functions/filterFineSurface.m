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
        depthMap = loadDepthMap(srcData(di,:), datasetIndex, depthList, radius);
        roughness = estiRoughness(depthMap);
%         if roughness > 0.4
%             roughness
%             validIndices(di) = 0;
%             drawPointCloud(srcData(di,:), datasetIndex, depthList, radius);
%             pause(10)
%         end
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

%----------------------------------------------
function depthMap = loadDepthMap(datarow, datasetIndex, depthList, radius_cm)
global dataIndices
model = struct('frame', datarow(dataIndices.frame), 'pixel', datarow(dataIndices.pixel), ...
                'point', datarow(dataIndices.point), 'normal', datarow(dataIndices.normal), ...
                'praxis', datarow(dataIndices.praxis));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/%s', datasetPath, depthList{model.frame,1});
[depthMap, boundbox] = loadDepthMapScaled(filename, model.pixel, radius_cm);
centerPixel = model.pixel - [boundbox(1) boundbox(3)] + [1 1];
% if abs(model.point(1) - depthMap(centerPixel(2), centerPixel(1))) > 0.01
%     ME = MException('shapeDistance:loadDepthMap', 'wrong depth map loaded');
%     throw(ME)
% ends

end

function drawPointCloud(datarow, datasetIndex, depthList, radius_cm)
global dataIndices
model = struct('frame', datarow(dataIndices.frame), 'pixel', datarow(dataIndices.pixel), ...
                'point', datarow(dataIndices.point), 'normal', datarow(dataIndices.normal), ...
                'praxis', datarow(dataIndices.praxis));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/%s', datasetPath, depthList{model.frame,1});
ptcloud = loadPointCloud(filename, model.pixel, radius_cm, model.point);
figure(1)
pcshow(ptcloud)
end

function roughness = estiRoughness(depthMap)
mapWidth = size(depthMap,2);
mapHeight = size(depthMap,1);
count = 0;
roughCount = 0;

for u=2:mapWidth-1
    for v=2:mapHeight-1
        roughH = roughnessFromThreeDepths(depthMap(u-1:u+1,v));
        roughV = roughnessFromThreeDepths(depthMap(u,v-1:v+1));
        if roughH~=0 && roughV~=0
            count = count+1;
            if min(roughH, roughV) > 0.003
                roughCount = roughCount+1;
            end
        end
    end
end
roughness = roughCount/count;
end

function roughness = roughnessFromThreeDepths(depths)
global deadDepth
roughness = 0;
if sum(depths>deadDepth) ~= 3
    return
end
roughness = abs(depths(2) - (depths(1)+depths(3))/2) / depths(2);
end
































