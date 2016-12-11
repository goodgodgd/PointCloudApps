function D2samplePoints(dsetIndex, radius, numSamples)

datasetPath = workingDir(dsetIndex, radius);
fileName = sprintf('%s/data.mat', datasetPath);
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    srcData = load(fileName);
end
srcData = srcData.data;
% srcData = srcData(1:5:end,:);

samplesRefer = sampleData(srcData, dsetIndex, radius, numSamples);
samplesQuery = sampleData(srcData, dsetIndex, radius, numSamples);

sampleFileName = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
save(sampleFileName, 'samplesRefer', 'samplesQuery');
'samples saved'
end

function samples = sampleData(srcData, dsetIndex, radius, numSamples)
numPreSamples = round(numSamples*1.1);
bsData = balanceSamples(srcData, numPreSamples);
samples = sampleReprstt(bsData, numPreSamples);
samples = filterFineSurface(samples, dsetIndex, radius);
filetered_samples_size = size(samples)
if size(samples,1) > numSamples
    samples = samples(1:numSamples,:);
end
end

