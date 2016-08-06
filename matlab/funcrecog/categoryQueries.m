function [queries gtcrp] = categoryQueries(categories, exceptInstanceIndices)

global numDescTypes bowFeatDim
queries = zeros(0, numDescTypes*bowFeatDim);
gtcrp = [];

for i=1:length(categories)
    numInstances = length(categories(i).instances);
    curExceptionIndices = modIndices(exceptInstanceIndices, numInstances);
    curInstanceIndices = 1:numInstances;
    curInstanceIndices = curInstanceIndices(~ismember(curInstanceIndices, curExceptionIndices));
    
    descriptors = getAllBoWFeats(categories(i).instances(curInstanceIndices));
    queries = [queries; descriptors];
    gtcrp = [gtcrp; ones(size(descriptors,1),1)*i];
end
