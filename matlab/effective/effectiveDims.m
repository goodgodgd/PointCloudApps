clc
clear

dsetIndices = 1:3;
radii = [4 6];

% for index = dsetIndices
%     for raius = radii
%         saveTotalDescs(index, raius);
%     end
% end

thresh = 0.0001;

for index = dsetIndices
    for raius = radii
        countZeros(index, raius);
        effectiveEigens(index, raius, thresh);
    end
end
