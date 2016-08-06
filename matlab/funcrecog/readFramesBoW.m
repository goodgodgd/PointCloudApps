function bowFrames = readFramesBoW(objIndices, frameLimit)

global dsetPath numDescTypes bowFeatDim
totalBoWDim = size(numDescTypes*bowFeatDim);
bowFrames = zeros(0, totalBoWDim);

for fi=1:frameLimit
    filename = sprintf('%s/OBJ%d_%d_%d_%d.txt', ...
                        dsetPath, objIndices(1), objIndices(2), objIndices(3), fi);
    if(exist(filename, 'file'))
        filename
        frameData = load(filename);
        bowFeat = BoWFeature(frameData);
        bowFrames = [bowFrames; bowFeat];
    end
end
