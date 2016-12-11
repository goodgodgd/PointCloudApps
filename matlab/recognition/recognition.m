clc
clear

addpath('functions')

radii = [4, 5, 6];
for radius = radii
    initGlobalsRecog(radius)
%     R1kmeansTrain(radius);
%     R2saveBowFeats(radius);
    R3instanceRecog(radius);
    R4categoryRecog(radius);
end
