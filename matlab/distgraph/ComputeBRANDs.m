function ComputeBRANDs(datasetPath, numSamples)

global brandLength
'ComputeBRANDs'
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;
referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);

referBrand = zeros(referSize, brandLength);
queryBrand = zeros(querySize, brandLength);

'reference sample BRAND'
tic
for i=1:referSize
    referBrand(i,:) = BRAND(datasetPath, samplesRefer(i,:));
end
toc

'query sample BRAND'
tic
for i=1:querySize
    queryBrand(i,:) = BRAND(datasetPath, samplesQuery(i,:));
end
toc

sampleFileName = sprintf('%s/BRAND_%d.mat', datasetPath, numSamples);
save(sampleFileName, 'referBrand', 'queryBrand');
end

