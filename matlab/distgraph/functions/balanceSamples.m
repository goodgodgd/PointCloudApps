function bsData = balanceSamples(srcData, numSamples)

bsData = srcData;
numCluts = round(numSamples*1.1);
iter = 0;

while(1)
    datalen = size(bsData,1);
    downSampleThresh = round(datalen/numCluts*4);
    downSampleNum = round(datalen/numCluts*2);
    sprintf('kmeans starts for %d descriptors', datalen)
    iter = iter+1
    time = clock;
    time(4:6)

    cluData = scaleForClustering(bsData);
    clutIndices = kmeans(cluData, numCluts, 'Distance', 'cityblock'...
                        , 'Start', 'cluster', 'EmptyAction', 'singleton');
    clutCounts = histc(clutIndices, 1:max(clutIndices));
    [sortedCounts, sortedIndices] = sort(clutCounts, 'descend');
    sortResult = [sortedCounts, sortedIndices]

    if sortedCounts(1) < downSampleThresh
        sprintf('largest cluster %d < %d', sortedCounts(1), downSampleThresh)
        break;
    end
    
    % sortedIndices: cluster index sorted by cluster size
    ci=1;
    while(sortedCounts(ci) >= downSampleThresh)
        % down sample cluster sortedIndices(ci)
        sprintf('cluster %d has %d, downsample to %d', sortedIndices(ci), sortedCounts(ci), downSampleNum)
        sampleData = bsData(clutIndices==sortedIndices(ci));
        sampleNum = size(sampleData,1);
        dsInterval = sampleNum/downSampleNum;
        if dsInterval < 1
            error('inappropriate down sampling')
        end
        dsIndices = round(1:dsInterval:sampleNum);
        downData = sampleData(dsIndices,:);
        
        otherData = bsData(clutIndices~=sortedIndices(ci));
        bsData = [downData; otherData];
        
        ci = ci+1;
    end
    sprintf('downsampled tracks: %d by %d', size(bsData,1), size(bsData,2))
end
