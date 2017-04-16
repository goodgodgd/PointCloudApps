function writeSamples()

clear
clc

addpath('../functions')
initGlobals

radii = [4 5 6];
numDsets = dataPath();
dsetIndices = 1:numDsets;
loopIndices = createLoopIndices(radii, dsetIndices)
loopLength = size(loopIndices,1);

for li = 1:loopLength
    dataIndex = loopIndices(li,2);
    radius = loopIndices(li,3);
    datasetPath = dataPath(dataIndex, radius)
    setCameraParams(datasetPath)

    reference = readReferenceSet(datasetPath);
    writeReferenceSamples(datasetPath, reference);
end
end

function loopIndices = createLoopIndices(radii, dsetIndices)
loopIndices = zeros(length(radii)*length(dsetIndices), 3);
loopcnt = 0;
for radius = radii
    for dataIndex = dsetIndices
        loopcnt = loopcnt+1;
        loopIndices(loopcnt,:) = [loopcnt dataIndex radius];
    end
end
end

function samplesRefer = readReferenceSet(datasetPath)
global numSamples
fileName = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    samples = load(fileName);
end
samplesRefer = samples.samplesRefer;
end

function writeReferenceSamples(datasetPath, samples)
global dataIndices
fid = fopen([datasetPath, '/sampleList.txt'], 'w');
if fid==-1
    error('cannot create sample list file')
end
sampleSize = size(samples,1)
for si=1:sampleSize
    frameIndex = samples(si,dataIndices.frame);
    pixel = samples(si,dataIndices.pixel);
    point = samples(si,dataIndices.point);
    filename = getDepthFileName(datasetPath, frameIndex);
    fprintf(fid, '%s %d %d %f %f %f\n', filename, pixel(1), pixel(2), point(1), point(2), point(3));
end
fclose(fid);
end
