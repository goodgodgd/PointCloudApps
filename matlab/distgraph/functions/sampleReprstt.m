function rpData = sampleReprstt(srcData, numSamples)

'sampleReprstt'
scDescs = scaleForClustering(srcData);
numCluts = round(numSamples*1.1);
[clutIndices, centroids, sumd, dists] ...
    = kmeans(scDescs, numCluts, 'Distance', 'cityblock', 'Start', 'cluster', 'EmptyAction', 'singleton');

% size(dists) = [N, 100], size(clutIndices) = [N, 1]
distInd = sub2ind(size(dists), 1:length(dists), clutIndices');
% extract distances to closest centroid, size(dists) = [N, 1]
dists = dists(distInd);

[size(clutIndices) size(dists) size(srcData)]
orderedData = [clutIndices dists' srcData];
% sort srcData w.r.t cluster index and distance to centroid
orderedData = sortrows(orderedData, [1 2]);
orderedData = orderedData(:,3:end);

% set output samples cloest to centroid
rpData = zeros(numCluts, size(srcData,2));
for si=1:numCluts
    clutData = orderedData(orderedData(:,1)==si);
    rpData(si,:) = clutData(1,:);
end
end

function scDescrs = scaleForClustering(srcData)

global dataIndices
pcwg = srcData(:,dataIndices.descrs(2));
fpfh = srcData(:,dataIndices.descrs(3));
shot = srcData(:,dataIndices.descrs(4));
tris = srcData(:,dataIndices.descrs(6));

scalePcwg = mean(sum(pcwg,2));
scaleFpfh = mean(sum(fpfh,2));
scaleShot = mean(sum(shot,2));
scaleTris = mean(sum(tris,2));

scDescrs = [pcwg/scalePcwg fpfh/scaleFpfh shot/scaleShot tris/scaleTris];

pcwgSize=4; spinSize=153; fpfhSize=33; shotSize=352; trisSize=spinSize*3;
pcwgidcs = 1:pcwgSize;
fpfhidcs = pcwgidcs(end)+1:pcwgidcs(end)+fpfhSize;
shotidcs = fpfhidcs(end)+1:fpfhidcs(end)+shotSize;
trisidcs = shotidcs(end)+1:shotidcs(end)+trisSize;
scaledDescMeans = [mean(sum(scDescrs(:,pcwgidcs), 2)) mean(sum(scDescrs(:,fpfhidcs), 2)) ...
    mean(sum(scDescrs(:,shotidcs), 2)) mean(sum(scDescrs(:,trisidcs), 2))]
end

