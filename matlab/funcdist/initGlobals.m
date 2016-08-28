global totalDimension; global eachDescIndices;

pcwgSize=4; spinSize=153; fpfhSize=33; shotSize=352;

pcwgIdcs = 1:pcwgSize;
fpfhIdcs = pcwgIdcs(end)+1:pcwgIdcs(end)+fpfhSize;
shotIdcs = fpfhIdcs(end)+1:fpfhIdcs(end)+shotSize;
spinIdcs = shotIdcs(end)+1:shotIdcs(end)+spinSize;
trisIdcs = shotIdcs(end)+1:shotIdcs(end)+spinSize*3;
totalDimension = trisIdcs(end);
eachDescIndices = containers.Map({1, 2, 3, 4, 5}, {pcwgIdcs, spinIdcs, fpfhIdcs, shotIdcs, trisIdcs});
