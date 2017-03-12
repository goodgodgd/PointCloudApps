function D4calcDescDists(datasetPath, numSamples)

global dataIndices gradWeight numDescTypes
'D4calcDescDists'
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

filename = sprintf('%s/BRAND_%d.mat', datasetPath, numSamples);
BRAND = load(filename);
brandRefer = BRAND.referBrand;
brandQuery = BRAND.queryBrand;

referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
assert(referSize==size(brandRefer,1) && querySize==size(brandQuery,1));
descrDists = zeros(referSize, querySize, numDescTypes);

% apply gradient weight
gradIndices = dataIndices.descrs(2);
gradIndices = gradIndices(3:4);
samplesRefer(:,gradIndices) = samplesRefer(:,gradIndices)*gradWeight;
samplesQuery(:,gradIndices) = samplesQuery(:,gradIndices)*gradWeight;

for ri=1:referSize
    for qi=1:querySize
        descrDists(ri,qi,1:6) = descrDistance(samplesRefer(ri,:), samplesQuery(qi,:));
        descrDists(ri,qi,7) = sum(abs(brandRefer(ri,:)-brandQuery(qi,:)));
    end
end

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
save(filename, 'descrDists');
'descriptor distances saved'
end

function descrDist = descrDistance(sampleRefer, sampleQuery)
global dataIndices
descrDist = zeros(1,6);
sampleDiffAbs = abs(sampleRefer - sampleQuery);
for dtype=1:6
    descrDist(dtype) = sum(sampleDiffAbs(dataIndices.descrs(dtype)));
end
end
