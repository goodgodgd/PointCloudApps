function bowFrames = readFramesBoW(objIndices, frameLimit, descIndices, descWords)

global numDescTypes bowFeatDim
bowFrames = zeros(0, numDescTypes*bowFeatDim);

for fi=1:frameLimit
    frameData = loadDescriptors([objIndices, fi+99]);
    if ~isempty(frameData)
        bowFeat = BoWFeature(frameData, descIndices, descWords);
        bowFrames = [bowFrames; bowFeat];
    end
end

function bowFeat = BoWFeature(frameData, descIndices, descWords)

global numDescTypes bowFeatDim
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

% normalize bow descriptor
for i=1:size(bowFeat,1)
    for k=1:numDescTypes
        indices = bowFeatDim*(k-1)+1:bowFeatDim*k;
        bowFeat(i,indices) = bowFeat(i,indices)/sum(bowFeat(i,indices));
    end
end
