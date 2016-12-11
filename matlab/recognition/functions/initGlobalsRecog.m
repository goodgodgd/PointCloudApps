function initGlobalsRecog(radius)

global numDescTypes bowFeatDim dataPath gradWeight
numDescTypes = 6;
bowFeatDim = 100;
dataPath = sprintf('E:/PaperData/rgbd-object-dataset/_DescR%d', radius);
gradWeight = 0.3;
end