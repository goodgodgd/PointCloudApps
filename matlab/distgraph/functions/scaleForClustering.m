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

scDescrs = [pcwg*scalePcwg fpfh*scaleFpfh shot*scaleShot tris*scaleTris];
