global totalDimension; global eachDescIndices;

cwgSize=4; spinSize=153; fpfhSize=33; shotSize=352;

cwgIdcs = 1:cwgSize;
spinIdcs = cwgIdcs(end)+1:cwgIdcs(end)+spinSize;
fpfhIdcs = spinIdcs(end)+1:spinIdcs(end)+fpfhSize;
shotIdcs = fpfhIdcs(end)+1:fpfhIdcs(end)+shotSize;
totalDimension = shotIdcs(end);
eachDescIndices = containers.Map({1, 2, 3, 4}, {cwgIdcs, spinIdcs, fpfhIdcs, shotIdcs});
