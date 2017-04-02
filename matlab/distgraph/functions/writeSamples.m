function writeSamples()

clear
clc

addpath('functions')
initGlobals
pause on

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
    
%     D5;
end

end

function loopIndices = createLoopIndices(radii, dsetIndices)
loopIndices = zeros(length(radii)*length(dsetIndices), 3);
loopcnt = 0;
for dataIndex = dsetIndices
    for radius = radii
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
fid = fopen([datasetPath, '/sampleList.txt'], 'w');
if fid==-1
    error('cannot create sample list file')
end
sampleSize = size(samples,1)
for si=1:sampleSize
    frameIndex = samples(si,1);
    pixel = samples(si,2:3);
    filename = getDepthFileName(datasetPath, frameIndex);
    fprintf(fid, '%s %d %d\n', filename, pixel(1), pixel(2));
end
fclose(fid);
end
