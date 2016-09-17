function effectiveEigens(dsetIndex, radius, thresh)

% clc
% clear
global eachDescIndices

addpath('..\funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex, radius)
numDescType = 6;

filename = sprintf('%s/totalDescs.mat', dsetPath);
totalDescs = load(filename);
totalDescs = totalDescs.totalDescs;

emptyvec = zeros(1,3);
eigenValues = containers.Map({1, 2, 3, 4, 5, 6}, ...
                                {emptyvec, emptyvec, emptyvec, emptyvec, emptyvec, emptyvec});

for dt=1:numDescType
    descriptors = totalDescs(:,eachDescIndices(dt));
    covar = cov(descriptors);
    eigvals = eig(covar);
    eigenValues(dt) = eigvals;
    [dt sum(eigvals>thresh) length(eigvals)]
end

filename = sprintf('%s/eigenValues.mat', dsetPath);
save(filename, 'eigenValues')
