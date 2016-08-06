function references = categoryReference(categories, instanceIndices)

global numDescTypes bowFeatDim
references = zeros(length(categories), numDescTypes*bowFeatDim);

for i=1:length(categories)
    numInstances = length(categories(i).instances);
    curInstIndices = modIndices(instanceIndices, numInstances);
    
    descriptors = getAllBoWFeats(categories(i).instances(curInstIndices));
    references(i,:) = mean(descriptors,1);
end
