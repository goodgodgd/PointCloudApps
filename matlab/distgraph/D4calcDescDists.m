function D4calcDescDists(datasetIndex, radius, numSamples)

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
descrDists = zeros(referSize, querySize, 6);

% apply gradient weight
global dataIndices
gradIndices = dataIndices.descrs(2);
gradIndices = gradIndices(3:4);
gradWeight = 0.3;
samplesRefer(:,gradIndices) = samplesRefer(:,gradIndices)*gradWeight;
samplesQuery(:,gradIndices) = samplesQuery(:,gradIndices)*gradWeight;

for ri=1:referSize
    for qi=1:querySize
        descrDists(ri,qi,:) = descrDistance(samplesRefer(ri,:), samplesQuery(qi,:));
    end
end

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
save(filename, 'descrDists');
'descriptor distances saved'
end

function descrDist = descrDistance(sampleRefer, sampleQuery)
global dataIndices
descrDist = zeros(6,1);
sampleDiffAbs = abs(sampleRefer - sampleQuery);
for dtype=1:6
    descrDist(dtype) = sum(sampleDiffAbs(dataIndices.descrs(dtype)));
end
end

