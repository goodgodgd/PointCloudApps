function R4categoryRecog(radius)

global dataPath numDescTypes bowFeatDim
addpath('functions')
initGlobalsRecog(radius)
minInstances = 5;
numRefSrc = 3;

filename = sprintf('%s/bowfeat_%d.mat', dataPath, bowFeatDim);
categories = load(filename);
categories = categories.categories;
a = arrayfun(@(n) length(n.instances) >= minInstances, categories);
categories = categories(a);
success = zeros(0, numDescTypes);

for refInstanceIndex = 1:minInstances
    refInstIndices = refInstanceIndex:refInstanceIndex+numRefSrc-1;
    referns = categoryReference(categories, refInstIndices);
    [queries gtcrp] = categoryQueries(categories, refInstIndices);
    size(queries);
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

'category recognition'
performance = mean(success,1)
end

function references = categoryReference(categories, instanceIndices)

global numDescTypes bowFeatDim
references = zeros(length(categories), numDescTypes*bowFeatDim);

for i=1:length(categories)
    numInstances = length(categories(i).instances);
    curInstIndices = modIndices(instanceIndices, numInstances);
    
    descriptors = getAllBoWFeats(categories(i).instances(curInstIndices));
    references(i,:) = mean(descriptors,1);
end
end

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
end
