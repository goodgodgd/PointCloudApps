function descriptors = getAllBoWFeats(instances)

global numDescTypes bowFeatDim
descriptors = zeros(0, numDescTypes*bowFeatDim);

for i=1:length(instances)
    for k=1:length(instances(i).videos)
        descriptors = [descriptors; instances(i).videos(k).frames];
    end
end

sprintf('num descriptors: %d', size(descriptors,1));
