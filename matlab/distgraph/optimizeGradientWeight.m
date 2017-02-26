function gradWeight = optimizeGradientWeight(datasetPath, numSamples)

global normalDistWeight
global dataIndices
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
shapeDists = shapeDists(:,:,1) + shapeDists(:,:,2)*normalDistWeight;
sdScale = 1000;
shapeDists = shapeDists*sdScale;
shapeDists = reshape(shapeDists,[],1);

filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samplesRefer = samples.samplesRefer;
samplesQuery = samples.samplesQuery;
referSize = size(samplesRefer,1);
querySize = size(samplesQuery,1);
descrDists = zeros(referSize,querySize,4);
pcwgIndices = dataIndices.descrs(2);

for ri=1:referSize
    for qi=1:querySize
        descrDists(ri,qi,:) = abs(samplesRefer(ri,pcwgIndices) - samplesQuery(qi,pcwgIndices));
    end
end
descrDists = reshape(descrDists,[],4);

gradWeight = gradientWeight(descrDists, shapeDists);
end

function weight = gradientWeight(descDiffAbs, shapeDists)
SDmat = repmat(shapeDists,1,4);
wdiff = descDiffAbs./SDmat;
len = size(wdiff,1);
warning('off');
cvx_begin quiet
    variables w(1,1) a(1,1);
%     minimize( norm(wdiff*[1 1 w w]' - a*ones(len,1), 1) );
    minimize( norm(wdiff*[a a w w]' - ones(len,1), 1) );
    subject to
        w >= 0;
        a >= 0;
cvx_end
warning('on');
grad = [w a mean(wdiff*[a a w w]' - ones(len,1))]
weight = w/a;
end
