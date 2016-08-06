function [success, bowmatch, bowdist, gtrank, gtdist] ...
        = compareBoW(referns, queries, gtcrp)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = size(descWords(1),1);
numInstances = size(referns,1);
numQueries = size(queries,1);
success = zeros(numQueries, numDescTypes);
bowmatch = zeros(numQueries, numDescTypes);
bowdist = zeros(numQueries, numDescTypes);
gtrank = zeros(numQueries, numDescTypes);
gtdist = zeros(numQueries, numDescTypes);
fsRatio = 1;

for i=1:numQueries
    for k=1:numDescTypes
        descIndices = vocSize*(k-1)+1:vocSize*k;
        diff = referns(:,descIndices) - ones(numInstances,1)*queries(i,descIndices);
        dist = sum(abs(diff),2);
        [sdist, sindices] = sort(dist);
        
        % rank of ground truth and distance
        gtidx = find(sindices==gtcrp(i));
        gtrank(i,k) = gtidx;
        gtdist(i,k) = sdist(gtidx);
        
        if sdist(1) < sdist(2)*fsRatio
            % bow feature matching result and distance
            bowmatch(i,k) = sindices(1);
            bowdist(i,k) = sdist(1);
            % success of bow feature matching
            success(i,k) = (sindices(1)==gtcrp(i));
        end
    end
end
