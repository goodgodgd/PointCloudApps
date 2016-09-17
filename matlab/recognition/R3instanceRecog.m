function R3instanceRecog(radius)
% clc
% clear
% radius = 4;

global dataPath numDescTypes bowFeatDim
addpath('functions')
initGlobalsRecog(radius)
minVideos = 3;

filename = sprintf('%s/bowfeat_%d.mat', dataPath, bowFeatDim);
categories = load(filename);
categories = categories.categories;
instances = [categories(:).instances];
success = zeros(0, numDescTypes);

for refVideoIndex = 2:minVideos
    referns = instanceReference(instances, refVideoIndex);
    [queries gtcrp] = instanceQueries(instances, [1 refVideoIndex]);
    [size(referns), size(queries)]
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

'instance recognition'
performance = mean(success,1)
