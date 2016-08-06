function [tracksBS] = balanceSamples(tracks, numCluts)

tracksBS = tracks;
trackMeansBS = meanOfTracks(tracksBS);
iter=0;

while(1)
    downSampleThresh = round(length(tracksBS)/numCluts*4);
    downSampleNum = round(length(tracksBS)/numCluts*2);
    sprintf('kmeans starts for %d by %d matrix', size(trackMeansBS,1), size(trackMeansBS,2))
    iter = iter+1
    time = clock

    clutIndices = kmeans(trackMeansBS, numCluts, 'Distance', 'cityblock', 'Start', 'cluster' ...
                        , 'EmptyAction', 'singleton');
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
        sampleTracks = tracksBS(clutIndices==sortedIndices(ci));
        sampleMeans = trackMeansBS(clutIndices==sortedIndices(ci), :);
        sampleIndices = clutIndices(clutIndices==sortedIndices(ci));
        
        otherTracks = tracksBS(clutIndices~=sortedIndices(ci));
        otherMeans = trackMeansBS(clutIndices~=sortedIndices(ci), :);
        otherIndices = clutIndices(clutIndices~=sortedIndices(ci));
        [downTracks, downMeans] = downSample(sampleTracks, sampleMeans, downSampleNum);
        size(downMeans)
        size(otherMeans)

        tracksBS = [downTracks; otherTracks];
        trackMeansBS = [downMeans; otherMeans];
        clutIndices = [sampleIndices(1:downSampleNum); otherIndices];
        if size(tracksBS,1)~=size(trackMeansBS,1)
            error('track data size not consistent %d %d', size(tracksBS,1), size(trackMeansBS,1))
        end
        
        ci = ci+1;
    end
    sprintf('downsampled tracks %d %d', size(tracksBS,1), size(trackMeansBS,1))
end

trackMeans = meanOfTracks(tracks);
for i=1:length(tracksBS)
    for k=1:length(tracks)
        if tracksBS(i).ID == tracks(k).ID
            if abs(trackMeansBS(i,1)-trackMeans(k,1)) > 0.0001 ...
                    || abs(trackMeansBS(i,2)-trackMeans(k,2)) > 0.0001
                error('inconsistently sampled')
            end
        end
    end
end
