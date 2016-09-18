function distance = shapeDistance(datasetIndex, depthList, radius, modelinfo, queryinfo)
modelinfo(1:6)
queryinfo(1:6)

global dataIndices
model = struct('frame', modelinfo(dataIndices.frame), 'pixel', modelinfo(dataIndices.pixel) ...
                , 'normal', modelinfo(dataIndices.normal), 'praxis', modelinfo(dataIndices.praxis));
query = struct('frame', queryinfo(dataIndices.frame), 'pixel', queryinfo(dataIndices.pixel) ...
                , 'normal', queryinfo(dataIndices.normal), 'praxis', queryinfo(dataIndices.praxis));
sprintf('normal praxis dot %f, %f', model.normal*model.praxis', query.normal*query.praxis');
datasetPath = workingDir(datasetIndex);
% load images within radius
depthFileName = sprintf('%s/%s', datasetPath, depthList{model.frame,1});
mPointCloud = readDepthImage(depthFileName, model.pixel, radius);
depthFileName = sprintf('%s/%s', datasetPath, depthList{query.frame,1});
qPointCloud = readDepthImage(depthFileName, query.pixel, radius);

'center point1'
[modelinfo(dataIndices.point); mPointCloud(:,1)'; queryinfo(dataIndices.point); qPointCloud(:,1)']

if isempty(mPointCloud) || isempty(qPointCloud)
    distance = -1;
    return
end

% roughly align two point cloud based on prior information
tgtRot = orthogonalAxes(model.normal, model.praxis)';
curRot = orthogonalAxes(query.normal, query.praxis)';
rotation = tgtRot/curRot;
aqPointCloud = rotation*qPointCloud;
translation = mPointCloud(:,1) - aqPointCloud(:,1);
aqPointCloud = aqPointCloud + repmat(translation, 1, size(qPointCloud,2));
sprintf('model %d frame %d points vs query %d frame %d points', ...
            model.frame, size(mPointCloud,2), query.frame, size(aqPointCloud,2))

% plot point clouds
figure(1);
subplot(1,2,1);
hold off;
plot3(mPointCloud(1,:),mPointCloud(2,:),mPointCloud(3,:),'bo', ...
        qPointCloud(1,:),qPointCloud(2,:),qPointCloud(3,:),'r.');
hold on;
drawAxes(mPointCloud(:,1), tgtRot);
drawAxes(qPointCloud(:,1), curRot);
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Red: model, blue: query');

subplot(1,2,2);
hold off;
plot3(mPointCloud(1,:),mPointCloud(2,:),mPointCloud(3,:),'bo', ...
        aqPointCloud(1,:),aqPointCloud(2,:),aqPointCloud(3,:),'r.');
hold on;
mNormal = [mPointCloud(:,1) mPointCloud(:,1)+model.normal'*0.3];
qNormal = [aqPointCloud(:,1) aqPointCloud(:,1)+rotation*query.normal'*0.3];
plot3(mNormal(1,:), mNormal(2,:), mNormal(3,:), 'b-', ...
            qNormal(1,:), qNormal(2,:), qNormal(3,:), 'r-');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('roughly aligned');

% icp distance
distance = 0;%alignedDist(mPointCloud, aqPointCloud);
end

function axes = orthogonalAxes(firstAxis, secondAxis)
lastAxis = cross(firstAxis, secondAxis);
secondAxis = cross(lastAxis, firstAxis);
axes = [firstAxis; secondAxis; lastAxis]';
end

function drawAxes(center, axes)
if size(center,1) > size(center,2)
    center = center'
end

points = [center; repmat(center,3,1) + axes*0.3];
plot3(points([1 2],1), points([1 2],2), points([1 2],3), 'r-')
plot3(points([1 3],1), points([1 3],2), points([1 3],3), 'g-')
plot3(points([1 4],1), points([1 4],2), points([1 4],3), 'b-')
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Red: model, blue: query');
end