function references = referInstance(instances, videoIndex)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = size(descWords(1),1);
numInstances = length(instances);
references = zeros(numInstances, vocSize*numDescTypes);

for i=1:numInstances
    references(i,:) = mean(instances(i).videos(videoIndex).frames, 1);
end
