clc
clear

addpath('..\distgraph\functions')
initGlobals

datasets = 1:3;
radii = [4];
nEPC = zeros(4,6,3);

for index = datasets
    for radius = radii
        config = [index radius]
        nEPC(:,:,index) = effectiveEigens(index, radius);
    end
end

nEPC1 = nEPC(:,:,1)
nEPC2 = nEPC(:,:,2)
nEPC3 = nEPC(:,:,3)