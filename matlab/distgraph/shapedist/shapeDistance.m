function distance = shapeDistance(datasetIndex, depthList, radius, modelinfo, queryinfo, drawFigure)

global dataIndices
model = struct('frame', modelinfo(dataIndices.frame), 'pixel', modelinfo(dataIndices.pixel), ...
                'point', modelinfo(dataIndices.point), 'normal', modelinfo(dataIndices.normal), ...
                'praxis', modelinfo(dataIndices.praxis));
query = struct('frame', queryinfo(dataIndices.frame), 'pixel', queryinfo(dataIndices.pixel), ...
                'point', modelinfo(dataIndices.point), 'normal', queryinfo(dataIndices.normal), ...
                'praxis', queryinfo(dataIndices.praxis));
sprintf('normal praxis dot %f, %f', model.normal*model.praxis', query.normal*query.praxis');

% load images within radius
datasetPath = workingDir(datasetIndex);
depthFileName = sprintf('%s/%s', datasetPath, depthList{model.frame,1});
pcModel = readDepthImage(depthFileName, model.pixel, radius);
depthFileName = sprintf('%s/%s', datasetPath, depthList{query.frame,1});
pcQuery = readDepthImage(depthFileName, query.pixel, radius);



% 'center point1'
% [modelinfo(dataIndices.point); pcModel(:,1)'; queryinfo(dataIndices.point); pcQuery(:,1)']

if size(pcModel,2) < 30 || size(pcQuery,2) < 30
    distance = -1;
    sprintf('insufficient points: %d, %d', size(pcModel,2), size(pcQuery,2))
    return
end

% roughly align two point cloud based on prior information
tgtRot = G2LRotation(model.normal, model.praxis)';
curRot = G2LRotation(query.normal, query.praxis)';
pcAlignedModel = tgtRot*pcModel;
pcAlignedQuery = curRot*pcQuery;
pcAlignedModel = pcAlignedModel - repmat(pcAlignedModel(:,1), 1, size(pcAlignedModel,2));
pcAlignedQuery = pcAlignedQuery - repmat(pcAlignedQuery(:,1), 1, size(pcAlignedQuery,2));
sprintf('model %d frame %d points vs query %d frame %d points', ...
            model.frame, size(pcAlignedModel,2), query.frame, size(pcAlignedQuery,2))

if drawFigure
    % plot point clouds
    fig = figure(1);
    clf(fig)
    subplot(1,3,1);
    hold on
    drawPointsWithAxes(pcModel, 'bo', tgtRot);
    hold off
    title('model');
    subplot(1,3,2);
    hold on
    drawPointsWithAxes(pcQuery, 'r.', curRot);
    hold off
    title('query');

    subplot(1,3,3);
    hold on;
    drawPointsWithAxes(pcAlignedModel, 'bo', eye(3));
    drawPointsWithAxes(pcAlignedQuery, 'r.', eye(3));
    title('roughly aligned');
    hold off;
end
% icp distance
distance = alignedDist(pcAlignedModel, pcAlignedQuery, drawFigure);
end

function axes = G2LRotation(firstAxis, secondAxis)
lastAxis = cross(firstAxis, secondAxis);
secondAxis = cross(lastAxis, firstAxis);
axes = [firstAxis; secondAxis; lastAxis]';
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
