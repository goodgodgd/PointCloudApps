function [sampleTracks, sampleMeans] = downSample(srcTracks, srcMeans, sampleNum)

trackCounts = zeros(length(srcTracks),1);
for i=1:length(srcTracks)
    trackCounts(i) = size(srcTracks.descriptors,1);
end
[val, index] = sort(trackCounts,'descend');
sampleTracks = srcTracks(index(1:sampleNum));
sampleMeans = srcMeans(index(1:sampleNum),:);
