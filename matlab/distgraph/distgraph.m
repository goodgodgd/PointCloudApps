clc
clear

addpath('functions')
initGlobals

datasets = 1:3;
radii = [4, 5, 6];
numSample = 100;

for radius = radii
    for index = datasets
        config = [radius index]
%         D1collectData(index, radius);
%         D2samplePoints(index, radius, numSample);
        D3calcShapeDists(index, radius, numSample);
        D4calcDescDists(index, radius, numSample);
        D5analyzeResult(index, radius, numSample);
% 
%         showDistances(index, radius, numSample);
%         showPointClouds(index, radius, numSample);
    end
end
