clc
clear

global radius
addpath('functions')
initGlobals

radius = 4;
dstasetIndex = 2;
datasetPath = dataPath(dstasetIndex, radius);

fileName = sprintf('%s/data.mat', datasetPath);
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    srcData = load(fileName);
end
srcData = srcData.data;
numData = size(srcData,1)

for i=1:numData
    try
        loadPCAligned(datasetPath, srcData(i,:));
    catch ME
        ME.identifier
        if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
            warning('%s %s', ME.identifier, ME.message);
        else
            rethrow(ME)
        end
    end
    pause
end

