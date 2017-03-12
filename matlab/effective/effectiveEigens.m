function nEPC = effectiveEigens(dsetIndex, radius)

global dataIndices
datasetPath = dataPath(dsetIndex, radius);
fileName = sprintf('%s/data.mat', datasetPath)
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    totalDescs = load(fileName);
end
totalDescs = totalDescs.data;
itv = floor(size(totalDescs,1)/10000);
if itv>1
    totalDescs = totalDescs(1:itv:end,:);
end

eachDescIndices = dataIndices.descrs;
nEPC = zeros(4,7);
                            
for dt=1:6
    descriptors = totalDescs(:,eachDescIndices(dt));
    covar = cov(descriptors);
    eigvals = eig(covar);
    nEPC(1,dt) = sum(eigvals > max(eigvals)/100);
    nEPC(2,dt) = size(descriptors,2);
    nEPC(3,dt) = nEPC(1,dt) / nEPC(2,dt) * 100;
    nEPC(4,dt) = max(eigvals);
end

% brandDescr = load('BRAND_total.mat');
% brandDescr = brandDescr.brandDescr;

totalSize = size(totalDescs,1)
brandDescr = zeros(totalSize,256);
'start computing BRAND'
for i=1:totalSize
    brandDescr(i,:) = BRAND(datasetPath, totalDescs(i,:), 0);
end
filename = [datasetPath, '/BRAND_total.mat'];
save(filename, 'brandDescr');

covar = cov(brandDescr);
eigvals = eig(covar);
nEPC(1,7) = sum(eigvals > max(eigvals)/100);
nEPC(2,7) = size(descriptors,2);
nEPC(3,7) = nEPC(1,7) / nEPC(2,7) * 100;
nEPC(4,7) = max(eigvals);
