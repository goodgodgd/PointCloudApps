function rpData = sampleReprstt(srcData, numSamples)

'sampleReprstt'
cluData = scaleForClustering(srcData);
[clutIndices, centroids, sumd, dists] ...
    = kmeans(cluData, numSamples, 'Distance', 'cityblock', 'Start', 'cluster', 'EmptyAction', 'singleton');

% size(dists) = [N, 100], size(clutIndices) = [N, 1]
distInd = sub2ind(size(dists), 1:length(dists), clutIndices');
% extract distances to closest centroid, size(dists) = [N, 1]
dists = dists(distInd);

[size(clutIndices) size(dists) size(srcData)]
orderedData = [clutIndices dists' srcData];
orderedData = sortrows(orderedData, [1 2]);

rpData = zeros(numSamples, size(srcData,2));

for si=1:numSamples
    cluData = orderedData(orderedData(:,1)==si, 3:end);
    rpData(si,:) = cluData(1,:);
end
