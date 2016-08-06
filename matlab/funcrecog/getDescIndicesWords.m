function [descIndices descWords] = getDescIndicesWords()

global numDescTypes; global dataPath;

cwgSize=4; spinSize=153; fpfhSize=33; shotSize=352;
cwgIdcs = 1:cwgSize;
spinIdcs = cwgIdcs(end)+1:cwgIdcs(end)+spinSize;
fpfhIdcs = spinIdcs(end)+1:spinIdcs(end)+fpfhSize;
shotIdcs = fpfhIdcs(end)+1:fpfhIdcs(end)+shotSize;
descIndices = containers.Map({1, 2, 3, 4}, {cwgIdcs, spinIdcs, fpfhIdcs, shotIdcs});

if nargout==1
    return
end

descWords = containers.Map({1, 2, 3, 4}, {0, 0, 0, 0});

for i=1:numDescTypes
    filename = sprintf('%s/word%d.mat', dataPath, i);
    words = load(filename);
    descWords(i) = words.words;
end
