function D3calcShapeDists(datasetIndex, radius, numSamples)

global dataIndices
totalDescLen = length(dataIndices.total);

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
sampleSize = size(samples,1);
totalSize = sampleSize*(sampleSize-1)/2;
distances = struct('shape', zeros(totalSize,1), 'descr', zeros(totalSize,totalDescLen), 'sample', zeros(totalSize,2));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

vidx = 0;
for ri=1:sampleSize-1
    for ci=ri+1:sampleSize
        shdist = shapeDistance(datasetIndex, depthList, radius, samples(ri,:), samples(ci,:), false);
        if shdist < 0
            continue
        end
        vidx = vidx+1;
        distances.shape(vidx) = shdist;
        
%         distances.descr(vidx,:) = samples(ri, dataIndices.total) - samples(ci, dataIndices.total);
        for ti=1:6
            distances.descr(vidx,ti) = descDistance(samples(ri,dataIndices.descrs(ti)) ...
                                                  , samples(ci,dataIndices.descrs(ti)), ti);
        end
        distances.sample(vidx,:) = [ri ci];
    end
end

distances.shape = distances.shape(1:vidx);
distances.descr = distances.descr(1:vidx,:);
if size(distances.shape,1) < size(distances.shape,2)
    distances.shape = distances.shape';
end
if size(distances.descr,1) < size(distances.descr,2)
    distances.descr = distances.descr';
end

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/distances_%d.mat', datasetPath, numSamples);
save(filename, 'distances');
'samples saved'
end

function distance = descDistance(descr1, descr2, typeindex)
L2types = 4;
if sum(L2types==typeindex) > 0
    distance = norm(descr1 - descr2, 2);
else
    distance = norm(descr1 - descr2, 1);
end
end
