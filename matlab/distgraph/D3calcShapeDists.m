function D3calcShapeDists(datasetIndex, radius, numSamples)

global dataIndices
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;
distances = struct('shape', zeros(totalSize,1), 'descr', zeros(totalSize,6));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath)
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};

vidx = 0;
for ri=1:sampleSize-1
    for ci=ri+1:sampleSize
        vidx = vidx+1;
        distances.shape(vidx) = shapeDistance(datasetIndex, depthList, radius ...
                                              , samples(ri,:), samples(ci,:));
        for ti=1:6
            distances.descr(vidx,ti) = descDistance(samples(ri,dataIndices.descrs(ti)) ...
                                                  , samples(ci,dataIndices.descrs(ti)));
        end
    end
end

filename = sprintf('%s/shapeDist_%d.mat', datasetPath, numSamples);
save(filename, 'shapeDist');
'samples saved'
