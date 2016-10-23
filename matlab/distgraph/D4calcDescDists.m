function D4calcDescDists(datasetIndex, radius, numSamples)

global dataIndices
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;

filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
indexPairs = shapeDists.indexPairs;
shapeDists = shapeDists.shapeDists;
totalSize = size(indexPairs,1);
descDists = zeros(totalSize, 7);

% compute difference between samples and distance between raw descriptors
sampleDiff = abs(samples(indexPairs(:,1),:) - samples(indexPairs(:,2),:));
for dtype=1:6
    descDists(:,dtype) = sum(sampleDiff(:, dataIndices.descrs(dtype)), 2);
end

% compute optimal gradient weight
distThresh = radius*1.5;
validIndices = (shapeDists < distThresh);
gradWeight = gradientWeight(sampleDiff(validIndices, dataIndices.descrs(dtype)), shapeDists(validIndices));

% compute optimized descriptor distance
pcwgDiff = sampleDiff(:, dataIndices.descrs(2));
descDists(:,7) = pcwgDiff*[1 1 gradWeight gradWeight]';

filename = sprintf('%s/descDists_%d.mat', datasetPath, numSamples);
save(filename, 'descDists');
'descriptor distances saved'

end

function weight = gradientWeight(descDiffs, shapeDists)

SDmat = repmat(shapeDists,1,4);
wdiff = descDiffs./SDmat;
len = size(wdiff,1);
cvx_begin quiet
    variables w(1,1) a(1,1);
    minimize( norm(wdiff*[1 1 w w]' - a*ones(len,1), 1) );
    subject to
        w >= 0;
        a >= 0;
cvx_end

grad = [w a norm(wdiff*[1 1 w w]' - a*ones(len,1), 1)]
weight = w;
end















