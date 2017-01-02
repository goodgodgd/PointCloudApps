function performance = R3instanceRecog(radius)

global dataPath numDescTypes bowFeatDim
minVideos = 3;

filename = sprintf('%s/bowfeat_%d.mat', dataPath, bowFeatDim);
categories = load(filename);
categories = categories.categories;
instances = [categories(:).instances];
success = zeros(0, numDescTypes);

for refVideoIndex = 2:minVideos
    referns = instanceReference(instances, refVideoIndex);
    [queries, gtcrp] = instanceQueries(instances, [1 refVideoIndex]);
%     [size(referns), size(queries)]
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

'instance recognition'
performance = mean(success,1);
end

function references = instanceReference(instances, videoIndex)

global numDescTypes bowFeatDim
numInstances = length(instances);
references = zeros(numInstances, bowFeatDim*numDescTypes);

for i=1:numInstances
%     [size(references), size(instances(i).videos(videoIndex).frames)]
    references(i,:) = mean(instances(i).videos(videoIndex).frames, 1);
end
end

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
end