function bowFeat = BoWFeature(frameData)

global eachDescIndices; global descWords;

numDescTypes = length(eachDescIndices);
vocSize = size(descWords(1),1);
bowFeat = zeros(1, vocSize*numDescTypes);

for i=1:size(frameData,1)
    for k=1:numDescTypes
        diff = descWords(k) - ones(vocSize,1)*frameData(i,eachDescIndices(k));
        dist = sqrt(sum(diff.*diff, 2));
        [sdist, sindices] = sort(dist);
        wordIndex = sindices(1) + vocSize*(k-1);
        bowFeat(wordIndex) = bowFeat(wordIndex)+1;
    end
end

for i=1:size(bowFeat,1)
    for k=1:numDescTypes
        indices = vocSize*(k-1)+1:vocSize*k;
        bowFeat(i,indices) = bowFeat(i,indices)/sum(bowFeat(i,indices));
    end
end
