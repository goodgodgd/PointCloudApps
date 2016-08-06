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


referns = referInstance(instances, 2);
[queries gtcrp] = queryInstance(instances, 2);
% 
% sum(abs(referns(1,:) - queries(1,:)))
% sum(abs(referns(15,:) - queries(1,:)))
% sum(abs(referns(1,:) - queries(2,:)))
% sum(abs(referns(15,:) - queries(2,:)))
% sum(abs(referns(15,:) - referns(1,:)))
% sum(abs(queries(1,:) - queries(2,:)))
numDescTypes = length(eachDescIndices);
vocSize = size(descWords(1),1);
numQueries = size(queries,1);
diff = zeros(numQueries,0);
for k=1:numDescTypes
    descIndices = vocSize*(k-1)+1:vocSize*k;
    tdiff = queries(:,descIndices) - ones(numQueries,1)*referns(1,descIndices);
    diff = [diff sum(abs(tdiff),2)];
end

for k=1:numDescTypes
    descIndices = vocSize*(k-1)+1:vocSize*k;
    tdiff = queries(:,descIndices) - ones(numQueries,1)*referns(15,descIndices);
    diff = [diff sum(abs(tdiff),2)];
end