function descriptors = getAllDescriptors(instances)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = length(descWords(1));
descriptors = zeros(0, vocSize*numDescTypes);

for i=1:length(instances)
    for k=1:length(instances(i).videos)
        descriptors = [descriptors; instances(i).videos(k).frames];
    end
end

sprintf('num descriptors: %d', size(descriptors,1));

