function D3calcShapeDists(datasetIndex, radius, numSamples)

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;
shapeDists = zeros(totalSize,2);
indexPairs = zeros(totalSize,2);

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

vidx = 0;
for ri=1:sampleSize-1
    sprintf('================== ri=%d =================', ri)
    for ci=ri+1:sampleSize
        shdist = 0;
        try
            shdist = shapeDistance(datasetIndex, depthList, radius, samples(ri,:), samples(ci,:), false);
        catch ME
            ME.identifier
            if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
                warning('%s %s', ME.identifier, ME.message);
            else
                rethrow(ME)
            end
        end
        vidx = vidx+1;
        shapeDists(vidx,:) = shdist;
        indexPairs(vidx,:) = [ri ci];
    end
end

meanDist = mean(shapeDists,1)
shapeDists = shapeDists(1:vidx,:);
indexPairs = indexPairs(1:vidx,:);

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
save(filename, 'shapeDists', 'indexPairs');
'shape distances saved'
end
