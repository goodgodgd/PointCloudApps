function drawPointCloud(ptcloud, numSamples, showNormal)

if numSamples > 0 && ptcloud.Count > numSamples
    drawIndices = randperm(ptcloud.Count, 100);
    ptcloud = select(ptcloud, drawIndices);
end

% plot point clouds
figure(1);
pcshow(ptcloud.Location, [1 0 0]);
if showNormal==1
    hold on
    quiver3(ptcloud.Location(:,1), ptcloud.Location(:,2), ptcloud.Location(:,3), ...
            ptcloud.Normal(:,1), ptcloud.Normal(:,2), ptcloud.Normal(:,3), 'r');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    hold off
end
end
