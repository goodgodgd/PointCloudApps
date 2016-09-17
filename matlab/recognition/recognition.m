clc
clear

radii = [3, 4, 5];
for radius = radii
%     R1kmeansTrain(radius);
%     R2saveBowFeats(radius);
    R3instanceRecog(radius);
    R4categoryRecog(radius);
end
