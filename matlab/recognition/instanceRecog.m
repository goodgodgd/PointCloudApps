clc
clear

global eachDescIndices; global descWords;

addpath('../utils')
initGlobals;
dsetPath = descPath()
descWords = loadWords();
minVideos = 3;

filename = sprintf('%s/bowfeat.mat', dsetPath);
categories = load(filename);
categories = categories.categories;
instances = [categories(:).instances];
numDescTypes = length(eachDescIndices);
success = zeros(0, numDescTypes);

for refVideoIndex = 2:minVideos
    referns = referInstance(instances, refVideoIndex);
    [queries gtcrp] = queryInstance(instances, refVideoIndex);
    [tsuccess, tbowmatch, tbowdist, tgtrank, tgtdist] = compareBoW(referns, queries, gtcrp);
    success = [success; tsuccess];
%     success2 = tgtrank==1;
%     loss = sum(sum(success2 - tsuccess))
end

performance = mean(success,1)
