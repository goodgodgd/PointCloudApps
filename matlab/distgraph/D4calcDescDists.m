function D4calcDescDists(datasetPath, numSamples)

global dataIndices gradWeight numDescTypes
'D4calcDescDists'
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
descrDists = zeros(referSize, querySize, numDescTypes);

% apply gradient weight
gradIndices = dataIndices.descrs(2);
gradIndices = gradIndices(3:4);
samplesRefer(:,gradIndices) = samplesRefer(:,gradIndices)*gradWeight;
samplesQuery(:,gradIndices) = samplesQuery(:,gradIndices)*gradWeight;

descrDists(1,1,7) = BRAND_Distance(datasetPath, samplesRefer(1,:), samplesQuery(22,:));

for ri=1:referSize
    for qi=1:querySize
        descrDists(ri,qi,1:6) = descrDistance(samplesRefer(ri,:), samplesQuery(qi,:));
    end
end

for ri=1:referSize
    computing_BRAND_dists_reference_index = ri
    referenceSample = samplesRefer(ri,:);
    tic
    for qi=1:querySize
        descrDists(ri,qi,7) = BRAND_Distance(datasetPath, referenceSample, samplesQuery(qi,:));
    end
    toc
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

function dist = BRAND_Distance(datasetPath, sampleRefer, sampleQuery)
brandRefer = BRAND(datasetPath, sampleRefer);
brandQuery = BRAND(datasetPath, sampleQuery);
dist = sum(abs(brandRefer-brandQuery));
end