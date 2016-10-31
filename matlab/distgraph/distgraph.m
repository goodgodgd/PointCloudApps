clc
clear

addpath('functions')
initGlobals

datasets = 1:3;
radii = [4, 6];
numSamples = 100;

for radius = radii
    for index = datasets
%         D1collectData(index, radius);
        D2samplePoints(index, radius, numSamples);
        D3calcShapeDists(index, radius, numSamples);
        D4calcDescDists(index, radius, numSamples);
%         D5analyzeResult(index, radius, numSamples);
    end
end
