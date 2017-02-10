function nEPC = effectiveEigens(dsetIndex, radius)

global dataIndices
datasetPath = workingDir(dsetIndex, radius);
fileName = sprintf('%s/data.mat', datasetPath);
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
nEPC = zeros(4,6);
                            
for dt=1:6
    descriptors = totalDescs(:,eachDescIndices(dt));
    covar = cov(descriptors);
    eigvals = eig(covar);
    nEPC(1,dt) = sum(eigvals > max(eigvals)/100);
    nEPC(2,dt) = size(descriptors,2);
    nEPC(3,dt) = nEPC(1,dt) / nEPC(2,dt) * 100;
    nEPC(4,dt) = max(eigvals);
end

