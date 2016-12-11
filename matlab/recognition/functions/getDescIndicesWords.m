function [descIndices, descWords] = getDescIndicesWords()

global numDescTypes dataPath bowFeatDim

prcvSize=2; pcwgSize=4; spinSize=153; fpfhSize=33; shotSize=352; trisSize=spinSize*3;
prcvIdcs = 1:prcvSize;
pcwgIdcs = 1:pcwgSize;
fpfhIdcs = pcwgIdcs(end)+1:pcwgIdcs(end)+fpfhSize;
shotIdcs = fpfhIdcs(end)+1:fpfhIdcs(end)+shotSize;
spinIdcs = shotIdcs(end)+1:shotIdcs(end)+spinSize;
trisIdcs = shotIdcs(end)+1:shotIdcs(end)+trisSize;

descIndices = containers.Map({1, 2, 3, 4, 5, 6}, {prcvIdcs, pcwgIdcs, spinIdcs, fpfhIdcs, shotIdcs, trisIdcs});

if nargout==1
    return
end

emptymat = [0.001 0; 0 0];
descWords = containers.Map({1, 2, 3, 4, 5, 6}, {emptymat, emptymat, emptymat, emptymat, emptymat, emptymat});

for i=1:numDescTypes
    filename = sprintf('%s/word%d_%d.mat', dataPath, i, bowFeatDim);
    words = load(filename);
    descWords(i) = words.words;
end
end