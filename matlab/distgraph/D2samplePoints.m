function D2samplePoints(datasetPath, numSamples)
'D2samplePoints'
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

samplesRefer = sampleData(srcData, datasetPath, numSamples);
srcData = excludeData(srcData, samplesRefer);
'reference data is excluded'
samplesQuery = sampleData(srcData, datasetPath, numSamples);

sampleFileName = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
save(sampleFileName, 'samplesRefer', 'samplesQuery');
'samples saved'
end

function samples = sampleData(srcData, datasetPath, numSamples)
numPreSamples = round(numSamples*1.3);
bsData = balanceSamples(srcData, numPreSamples);
samples = sampleReprstt(bsData, numPreSamples);
samples = filterFineSurface(samples, datasetPath);
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






