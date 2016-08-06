clc
clear

global dataPath numDescTypes

addpath('../funcrecog')
initGlobals;
minInstances = 5;
numRefSrc = 3;

filename = sprintf('%s/bowfeat.mat', dataPath);
categories = load(filename);
categories = categories.categories;
a = arrayfun(@(n) length(n.instances) >= minInstances, categories);
categories = categories(a);
success = zeros(0, numDescTypes);

for refInstanceIndex = 1:minInstances
    refInstIndices = refInstanceIndex:refInstanceIndex+numRefSrc-1
    referns = categoryReference(categories, refInstIndices);
    [queries gtcrp] = categoryQueries(categories, refInstIndices);
    size(queries)
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

performance = mean(success,1)
