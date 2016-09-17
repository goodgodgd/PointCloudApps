clc
clear

addpath('functions')
addpath('shapedist')
initGlobals

datasets = 1:3;
radii = [4, 6];
numSamples = 500;

for index = datasets
    for radius = radii
%         D1collectData(index, radius);
%         D2samplePoints(index, radius, numSamples);
        D3calcShapeDists(index, radius, numSamples);
    end
end
