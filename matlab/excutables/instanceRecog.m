clc
clear

global dataPath numDescTypes bowFeatDim

addpath('../funcrecog')
initGlobals;
minVideos = 3;

filename = sprintf('%s/bowfeat.mat', dataPath);
categories = load(filename);
categories = categories.categories;
instances = [categories(:).instances];
success = zeros(0, numDescTypes);

for refVideoIndex = 2:minVideos
    referns = instanceReference(instances, refVideoIndex);
    [queries gtcrp] = instanceQueries(instances, [1 refVideoIndex]);
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

performance = mean(success,1)
