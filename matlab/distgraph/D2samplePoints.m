function D2samplePoints(dsetIndex, radius, numSamples)

datasetPath = workingDir(dsetIndex, radius);
descrFileName = sprintf('%s/data.mat', datasetPath);
if exist(descrFileName, 'file')==0
    error('data file does not exist: %s', descrFileName)
else
    srcData = load(descrFileName);
end

srcData = srcData.data;
numPreSamples = round(numSamples*1.1);
bsData = balanceSamples(srcData, numPreSamples);
samples = sampleReprstt(bsData, numPreSamples, dsetIndex);
samples = filterFineSurface(samples, dsetIndex, radius, numSamples);

sampleFileName = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
save(sampleFileName, 'samples');
'samples saved'
