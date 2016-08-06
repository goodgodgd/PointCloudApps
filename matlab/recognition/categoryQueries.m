function [queries gtcrp] = categoryQueries(categories, exceptInstanceIndices)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = length(descWords(1));
queries = zeros(0, vocSize*numDescTypes);
gtcrp = [];

for i=1:length(categories)
    numInstances = length(categories(i).instances);
    needMod = (exceptInstanceIndices > numInstances);
    curExceptionIndices = exceptInstanceIndices;
    curExceptionIndices(needMod) = mod(curExceptionIndices(needMod), numInstances);
    curInstanceIndices = 1:numInstances;
    curInstanceIndices = curInstanceIndices(~ismember(curInstanceIndices, curExceptionIndices))
    descriptors = getAllDescriptors(categories(i).instances(curInstanceIndices));
    queries = [queries; descriptors];
    gtcrp = [gtcrp; ones(size(descriptors,1),1)*i];
end

