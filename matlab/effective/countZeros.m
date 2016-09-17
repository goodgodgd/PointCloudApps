function countZeros(dsetIndex, radius)

% clc
% clear
global eachDescIndices

addpath('..\funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex, radius)
numDescType = 6;

filename = sprintf('%s/totalDescs.mat', dsetPath);
totalDescs = load(filename);
totalDescs = totalDescs.totalDescs;

numFrames = size(totalDescs,1);
meanZeros = zeros(1,numDescType);

for dt=1:numDescType
    descriptors = totalDescs(:,eachDescIndices(dt));
    zeroCount = sum(sum(descriptors==0));
    meanZeros(dt) = zeroCount/numFrames;
end

meanZeros
filename = sprintf('%s/meanZeros.mat', dsetPath);
save(filename, 'meanZeros')
