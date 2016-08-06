function rpstTracks = sampleReprstt(tracks, numCluts)

trackMeans = meanOfTracks(tracks);

[clutIndices, centroids, sumd, dists] ...
    = kmeans(trackMeans, numCluts, 'Distance', 'cityblock', 'Start', 'cluster', 'EmptyAction', 'singleton');

clutCounts = histc(clutIndices, 1:max(clutIndices));
[sortedCounts, sortedIndices] = sort(clutCounts, 'descend');
sortResult = [sortedCounts, sortedIndices]

'sampleReprstt'
% size(dists) = [N, 100]
% size(clutIndices) = [N, 1]
distInd = sub2ind(size(dists), 1:length(dists), clutIndices');
dists = dists(distInd); % distance to closest centroid
cidcDist = [clutIndices, dists', (1:length(clutIndices))'];
% sort by cluster index first and then by distance second
cidcDist = sortrows(cidcDist, [1, 2]);

closeTrackIDs = zeros(numCluts,1);
farTrackIDs = zeros(numCluts,1);
for i=1:numCluts
    % extract [clusterIndex, distance, trackID] for specific cluster sorted
    % by distance
    distIidc = cidcDist(cidcDist(:,1)==i,:);
    closeTrackIDs(i) = distIidc(1,3);
    farIndex = max([1,floor(size(distIidc,1)/2)]);
%     sprintf('cluster size %d', size(distIidc,1))
    farTrackIDs(i) = distIidc(farIndex,3);
end

% [closeIndices farIndices]
% rpstTracks = repmat(tracks(1), numCluts*2, 1);
% rpstTracks(1:2:numCluts*2) = tracks(closeIndices(:,3));
% rpstTracks(2:2:numCluts*2) = tracks(farIndices(:,3));

rpstTracks = tracks(closeTrackIDs);
