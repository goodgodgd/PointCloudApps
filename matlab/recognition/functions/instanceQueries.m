function [queries, gtcrp] = instanceQueries(instances, exceptionVideoIndices)

global numDescTypes bowFeatDim

numInstances = length(instances);
queries = zeros(0, numDescTypes*bowFeatDim);
gtcrp = zeros(0, 1);

for i=1:numInstances
    numVideos = length(instances(i).videos);
    videoIndices = 1:numVideos;
    boolQuery = ~ismember(videoIndices, exceptionVideoIndices);
    for k=1:numVideos
        if boolQuery(k)==1
            videoFrames = instances(i).videos(k).frames;
            queries = [queries; videoFrames];
            gtcrp = [gtcrp; ones(size(videoFrames,1),1)*i];
        end
    end
end
