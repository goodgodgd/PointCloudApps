function distance = shapeDistance(datasetIndex, depthList, radius, modelinfo, queryinfo, drawFigure)

global dataIndices
model = struct('frame', modelinfo(dataIndices.frame), 'pixel', modelinfo(dataIndices.pixel), ...
                'point', modelinfo(dataIndices.point), 'normal', modelinfo(dataIndices.normal), ...
                'praxis', modelinfo(dataIndices.praxis));
query = struct('frame', queryinfo(dataIndices.frame), 'pixel', queryinfo(dataIndices.pixel), ...
                'point', queryinfo(dataIndices.point), 'normal', queryinfo(dataIndices.normal), ...
                'praxis', queryinfo(dataIndices.praxis));
sprintf('normal praxis dot %f, %f', model.normal*model.praxis', query.normal*query.praxis');
    
% load point cloud within radius
datasetPath = workingDir(datasetIndex);
depthFileName = sprintf('%s/%s', datasetPath, depthList{model.frame,1});
pcModel = loadPointCloud(depthFileName, model.pixel, radius, model.point);
depthFileName = sprintf('%s/%s', datasetPath, depthList{query.frame,1});
pcQuery = loadPointCloud(depthFileName, query.pixel, radius, query.point);

% sprintf('model %d frame %d points vs query %d frame %d points', ...
%             model.frame, pcModel.Count, query.frame, pcQuery.Count)

% roughly align two point cloud based on prior information
% get transformation for center to be origin with aligned axes
tformModel = transformG2L(model.point, model.normal, model.praxis);
tformQuery = transformG2L(query.point, query.normal, query.praxis);
pcModelAligned = pctransform(pcModel, tformModel);
pcQueryAligned = pctransform(pcQuery, tformQuery);

% point cloud REGISTRATION
[tformReg, pcQueryReg] = pcregrigid(pcQueryAligned, pcModelAligned, ...
                                    'Metric', 'pointToPoint', 'InlierRatio', 1);
% point to plane distance
distance = pointToPlaneDist(pcModelAligned, pcQueryReg);

if drawFigure && distance(1) < 0.002
    distance
    drawPointClouds(pcModel, pcQuery, pcModelAligned, pcQueryAligned, pcQueryReg, distance);
    pause
end
end

function tform = transformG2L(center, firstAxis, secondAxis)
firstAxis = firstAxis/norm(firstAxis);
lastAxis = cross(firstAxis, secondAxis);
lastAxis = lastAxis/norm(lastAxis);
secondAxis = cross(lastAxis, firstAxis);
secondAxis = secondAxis/norm(secondAxis);

% global to local rotation
g2lrot = [firstAxis; secondAxis; lastAxis];
% local to global rotation
l2grot = g2lrot';
% local to global transformation
tfmat = [l2grot reshape(center,3,1); 0 0 0 1];
% global to local transformation
tfmat = inv(tfmat);
tform = affine3d(tfmat');
end

function drawPointClouds(pcModel, pcQuery, pcModelAligned, pcQueryAligned, pcQueryReg, distance)
% plot point clouds
fig = figure(1);
clf(fig)
subplot(2,2,1);
pcshow(pcModel.Location);
title('raw model cloud')
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(2,2,2);
pcshow(pcQuery.Location);
title('raw query cloud')
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(2,2,3);
pcshow(pcModelAligned.Location, [1 0 0]);
hold on
pcshow(pcQueryAligned.Location, [0 0 1]);
title('roughly aligned clouds')
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off

subplot(2,2,4);
pcshow(pcModelAligned.Location, [1 0 0]);
hold on
pcshow(pcQueryReg.Location, [0 0 1]);
title(sprintf('precisely aligned clouds %.4f', distance(1)))
xlabel('X'); ylabel('Y'); zlabel('Z');
hold off

end

function drawPointsWithAxes(points, option, axes)
plot3(points(1,:),points(2,:),points(3,:), option);
center = points(:,1)';
axisLines = [center; repmat(center,3,1) + axes*0.1];
plot3(axisLines([1 2],1), axisLines([1 2],2), axisLines([1 2],3), 'r-')
plot3(axisLines([1 3],1), axisLines([1 3],2), axisLines([1 3],3), 'g-')
plot3(axisLines([1 4],1), axisLines([1 4],2), axisLines([1 4],3), 'b-')
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
end

function distance = pointToPlaneDist(pcmodel, pcquery)
if pcquery.Count < pcmodel.Count
    distance = pointToPlaneDist2(pcmodel, pcquery);
else
    distance = pointToPlaneDist2(pcquery, pcmodel);
end
end

function distance = pointToPlaneDist2(pcrefer, pccompa)
pdistsum = 0;
ndistsum = 0;
for cidx=1:pccompa.Count
    [ridx, ~] = findNearestNeighbors(pcrefer, pccompa.Location(cidx,:), 1);
    pdistsum = pdistsum + abs(dot(pccompa.Normal(cidx,:), ...
                            pccompa.Location(cidx,:) - pcrefer.Location(ridx,:)));
    ndistsum = ndistsum + acos(dot(pccompa.Normal(cidx,:), pcrefer.Normal(ridx,:)));
end
distance = [pdistsum/pccompa.Count ndistsum/pccompa.Count];
end















