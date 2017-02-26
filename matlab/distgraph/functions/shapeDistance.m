function distance = shapeDistance(datasetPath, modelinfo, queryinfo, drawFigure)

pcModelAligned = loadPCAligned(datasetPath, modelinfo);
pcQueryAligned = loadPCAligned(datasetPath, queryinfo);
% point cloud REGISTRATION
[tformReg, pcQueryReg] = pcregrigid(pcQueryAligned, pcModelAligned, ...
                                    'Metric', 'pointToPoint', 'InlierRatio', 1);
% point to plane distance
distance = distanceBetweenClouds(pcModelAligned, pcQueryReg);
if drawFigure && distance(1) < 0.002
    distance
    drawPointClouds(pcModel, pcQuery, pcModelAligned, pcQueryAligned, pcQueryReg, distance);
    pause
end
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

