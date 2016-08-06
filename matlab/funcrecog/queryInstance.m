function [queries, gtcrp] = queryInstance(instances, exceptionVideoIndex)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = size(descWords(1),1);
numInstances = length(instances);
queries = zeros(0, vocSize*numDescTypes);
gtcrp = zeros(0, 1);

for i=1:numInstances
    numVideos = length(instances(i).videos);
    queryVideoIndices = (1:numVideos~=1) & (1:numVideos~=exceptionVideoIndex);
%     queryVideoIndices = (1:numVideos~=exceptionVideoIndex);
    for k=1:numVideos
        if queryVideoIndices(k)~=0
            videoFrames = instances(i).videos(k).frames;
            queries = [queries; videoFrames];
            gtcrp = [gtcrp; ones(size(videoFrames,1),1)*i];
        end
    end
end
