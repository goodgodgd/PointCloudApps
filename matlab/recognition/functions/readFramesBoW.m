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
