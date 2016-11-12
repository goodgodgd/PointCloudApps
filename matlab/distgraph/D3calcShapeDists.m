function D3calcShapeDists(datasetIndex, radius, numSamples)

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
shapeDists = zeros(referSize, querySize, 2);

for ri=1:referSize
    for qi=1:querySize
        ri_qi = [ri qi]
        try
            shapeDists(ri,ci,:) = shapeDistance(datasetIndex, depthList, radius, ...
                                                samples(ri,:), samples(qi,:), false);
        catch ME
            ME.identifier
            if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
                warning('%s %s', ME.identifier, ME.message);
            else
                rethrow(ME)
            end
        end
    end
end

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
save(filename, 'shapeDists');
'shape distances saved'
end
