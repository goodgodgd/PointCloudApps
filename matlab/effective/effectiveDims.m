clc
clear

addpath('../distgraph/functions')
initGlobals

datasets = 1:3;
radii = [5];
nEPC = zeros(4,7,3);

for index = datasets
    for radius = radii
        config = [index radius]
        datasetPath = dataPath(index, radius);
        setCameraParams(datasetPath);
        nEPC(:,:,index) = effectiveEigens(index, radius);
    end
end

nEPC1 = nEPC(:,:,1)
nEPC2 = nEPC(:,:,2)
nEPC3 = nEPC(:,:,3)