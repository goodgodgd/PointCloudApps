function collectData(dsetIndex, radius)

% clc
% clear
global totalDimension eachDescIndices

addpath('..\funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex, radius)
descBegin = 9;
totalDescs = zeros(0, totalDimension);
numDescType = 6;

for frameCount=2:2:2000
    filename = sprintf('%s/DDS_%05d.txt', dsetPath, frameCount);
    if(exist(filename, 'file')==0)
        break;
    end
    data = load(filename);
    data = data(:, descBegin:end);
    totalDescs = [totalDescs; data];
    if mod(frameCount, 100)==0
        frameCount
        totalSize = size(totalDescs)
    end
end

totalSize = size(totalDescs)
filename = sprintf('%s/totalDescs.mat', dsetPath);
save(filename, 'totalDescs')
