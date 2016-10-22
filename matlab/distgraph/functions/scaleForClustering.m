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
