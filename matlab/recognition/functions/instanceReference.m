function references = instanceReference(instances, videoIndex)

global numDescTypes bowFeatDim
numInstances = length(instances);
references = zeros(numInstances, bowFeatDim*numDescTypes);

for i=1:numInstances
%     [size(references), size(instances(i).videos(videoIndex).frames)]
    references(i,:) = mean(instances(i).videos(videoIndex).frames, 1);
end
