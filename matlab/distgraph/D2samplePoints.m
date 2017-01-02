function D2samplePoints(dsetIndex, radius, numSamples)

datasetPath = workingDir(dsetIndex, radius);
fileName = sprintf('%s/data.mat', datasetPath);
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    srcData = load(fileName);
end
srcData = srcData.data;
itv = floor(size(srcData,1)/30000);
if itv>1
    srcData = srcData(1:itv:end,:);
end

samplesRefer = sampleData(srcData, dsetIndex, radius, numSamples);
srcData = excludeData(srcData, samplesRefer);
'reference data is excluded'
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

function dstData = excludeData(srcData, excludeData)
dstData = srcData;
excludeLen = size(excludeData,1);
for ei=1:excludeLen
    idx = dstData(:,1)==excludeData(ei,1) & dstData(:,2)==excludeData(ei,2) ...
            & dstData(:,3)==excludeData(ei,3);
    dstData(idx,:) = [];
end
sizebeforeafter = [size(srcData), size(excludeData,1), size(dstData,1)]
% assert(size(srcData,1) + size(excludeData,1) == size(dstData,1));
end






