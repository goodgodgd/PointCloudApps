function rpData = sampleReprstt(srcData, numSamples)

'sampleReprstt'
scDescs = scaleForClustering(srcData);
[clutIndices, centroids, sumd, dists] ...
    = kmeans(scDescs, numSamples, 'Distance', 'cityblock', 'EmptyAction', 'singleton');

% size(dists) = [N, 100], size(clutIndices) = [N, 1]
distInd = sub2ind(size(dists), 1:length(dists), clutIndices');
% extract distances to closest centroid, size(dists) = [N, 1]
dists = dists(distInd);

orderedData = [clutIndices dists' srcData];
% sort srcData w.r.t cluster index and distance to centroid
orderedData = sortrows(orderedData, [1 2]);

% set output samples cloest to centroid
rpData = zeros(numSamples, size(srcData,2));
for si=1:numSamples
    clutData = orderedData(orderedData(:,1)==si, 3:end);
    rpData(si,:) = clutData(1,:);
end
end


