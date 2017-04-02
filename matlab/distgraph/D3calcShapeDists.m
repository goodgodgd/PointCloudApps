function D3calcShapeDists(datasetPath, numSamples)
'D3calcShapeDists'
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;

referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
shapeDists = zeros(referSize, querySize, 2);

for ri=1:referSize
    referenceSample = samplesRefer(ri,:);
    tic
    for qi=1:querySize
        try
            shapeDists(ri,qi,:) = shapeDistance(datasetPath, ...
                                    referenceSample, samplesQuery(qi,:), false);
        catch ME
            ME.identifier
            if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
                warning('%s %s', ME.identifier, ME.message);
            else
                rethrow(ME)
            end
        end
    end
    eltime = toc;
    sprintf('reference index=%d, elapsed time=%f', ri, eltime)
end

filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
save(filename, 'shapeDists');
'shape distances saved'
end
