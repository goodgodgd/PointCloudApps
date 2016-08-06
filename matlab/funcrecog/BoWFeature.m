function bowFeat = BoWFeature(frameData)

global numDescTypes bowFeatDim
persistent descIndices descWords
[tdescIndices tdescWords] = getDescIndicesWords();
if isempty(descIndices)
    descIndices = tdescIndices;
    descWords = tdescWords;
end

bowFeat = zeros(1, bowFeatDim*numDescTypes);

for i=1:size(frameData,1)
    for k=1:numDescTypes
        diff = descWords(k) - ones(bowFeatDim,1)*frameData(i,descIndices(k));
        dist = sqrt(sum(diff.*diff, 2));
        [sdist, sindices] = sort(dist);
        wordIndex = sindices(1) + bowFeatDim*(k-1);
        bowFeat(wordIndex) = bowFeat(wordIndex)+1;
    end
end

for i=1:size(bowFeat,1)
    for k=1:numDescTypes
        indices = bowFeatDim*(k-1)+1:bowFeatDim*k;
        bowFeat(i,indices) = bowFeat(i,indices)/sum(bowFeat(i,indices));
    end
end
