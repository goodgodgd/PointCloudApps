function bowFrames = readFramesBoW(dsetPath, indices, frameLimit)

global eachDescIndices; global descWords;

totalBoWDim = size(descWords(1),1)*length(eachDescIndices);
bowFrames = zeros(0, totalBoWDim);

for fi=1:frameLimit
    filename = sprintf('%s/OBJ%d_%d_%d_%d.txt', dsetPath, indices(1), indices(2), indices(3), fi);
    if(exist(filename, 'file'))
        filename
        frameData = load(filename);
        bowFeat = BoWFeature(frameData);
        bowFrames = [bowFrames; bowFeat];
    end
end
