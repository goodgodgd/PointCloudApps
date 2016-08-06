function references = instanceReference(instances, videoIndex)

global numDescTypes bowFeatDim
numInstances = length(instances);
references = zeros(numInstances, bowFeatDim*numDescTypes);

for i=1:numInstances
    references(i,:) = mean(instances(i).videos(videoIndex).frames, 1);
end
