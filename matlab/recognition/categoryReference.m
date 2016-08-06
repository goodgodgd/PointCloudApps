function references = categoryReference(categories, instanceIndices)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = length(descWords(1));
references = zeros(length(categories), vocSize*numDescTypes);

for i=1:length(categories)
    numInstances = length(categories(i).instances);
    needMod = (instanceIndices > numInstances);
    curInstIndices = instanceIndices;
    curInstIndices(needMod) = mod(curInstIndices(needMod), numInstances)
    descriptors = getAllDescriptors(categories(i).instances(curInstIndices));
    references(i,:) = mean(descriptors,1);
end
