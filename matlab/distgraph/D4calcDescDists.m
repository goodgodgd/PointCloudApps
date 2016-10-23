function D4calcDescDists(datasetIndex, radius, numSamples)

global dataIndices
global distThreshForGradWeight
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;

filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
indexPairs = shapeDists.indexPairs;
shapeDists = shapeDists.shapeDists;
totalSize = size(indexPairs,1);
descrDists = zeros(totalSize, 7);

% compute difference between samples and distance between raw descriptors
sampleDiffAbs = abs(samples(indexPairs(:,1),:) - samples(indexPairs(:,2),:));
for dtype=1:6
    descrDists(:,dtype) = sum(sampleDiffAbs(:, dataIndices.descrs(dtype)), 2);
end

% compute optimal gradient weight
pcwg = 2;
validIndices = (shapeDists < distThreshForGradWeight(radius));
gradWeight = gradientWeight(sampleDiffAbs(validIndices, dataIndices.descrs(pcwg)), ...
                            shapeDists(validIndices));
gradWeight=0.5
% compute optimized descriptor distance
descrDists(:,7) = sampleDiffAbs(:, dataIndices.descrs(pcwg))*[1 1 gradWeight gradWeight]';

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
save(filename, 'descrDists');
'descriptor distances saved'

end

function weight = gradientWeight(descDiffAbs, shapeDists)
SDmat = repmat(shapeDists,1,4);
wdiff = descDiffAbs./SDmat;
len = size(wdiff,1);
warning('off');
cvx_begin quiet
    variables w(1,1) a(1,1);
%     minimize( norm(wdiff*[1 1 w w]' - a*ones(len,1), 1) );
    minimize( norm(wdiff*[a a w w]' - ones(len,1), 1) );
    subject to
        w >= 0;
        a >= 0;
cvx_end
warning('on');
grad = [w a mean(wdiff*[a a w w]' - ones(len,1))]
weight = w/a;
end















