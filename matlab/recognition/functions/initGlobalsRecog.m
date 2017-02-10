function initGlobalsRecog(radius, numWords)

global numDescTypes bowFeatDim dataPath rawDataPath gradWeight
numDescTypes = 6;
bowFeatDim = numWords;
rawDataPath = sprintf('E:/PaperData/rgbd-object-dataset/_DescR%d', radius);
dataPath = sprintf('E:/PaperData/rgbd-object-dataset/_DescR%d/session2', radius);
gradWeight = 0.3;
end