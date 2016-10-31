function bowFrames = readFramesBoW(objIndices, frameLimit)

global dataPath numDescTypes bowFeatDim
bowFrames = zeros(0, numDescTypes*bowFeatDim);

for fi=1:frameLimit
    filename = sprintf('%s/OBJ_C%02dI%02dV%02dF%03d.txt', ...
                        dataPath, objIndices(1), objIndices(2), objIndices(3), fi+99);
    if(exist(filename, 'file'))
        filename
        frameData = load(filename);
        bowFeat = BoWFeature(frameData);
        bowFrames = [bowFrames; bowFeat];
%     else
%         sprintf('NonExisting %s', filename)
    end
end

function bowFeat = BoWFeature(frameData)

global numDescTypes bowFeatDim
persistent descIndices descWords
if isempty(descIndices)
    [tdescIndices, tdescWords] = getDescIndicesWords();
    descIndices = tdescIndices;
    descWords = tdescWords;
end

bowFeat = zeros(1, bowFeatDim*numDescTypes);

% count closest words
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
