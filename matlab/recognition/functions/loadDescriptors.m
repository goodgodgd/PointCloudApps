function descriptors = loadDescriptors(fileSpecifier)

global rawDataPath gradWeight
descriptors = [];
if isnumeric(fileSpecifier)
    filename = sprintf('%s/OBJ_C%02dI%02dV%02dF%03d.txt', rawDataPath, ...
                        fileSpecifier(1), fileSpecifier(2), fileSpecifier(3), fileSpecifier(4));
else
    filename = sprintf('%s/%s', rawDataPath, fileSpecifier);
end

if(exist(filename, 'file'))
    filename
    descriptors = load(filename);
    descIndices = getDescIndicesWords();
    pcwgIndices = descIndices(2);
    gradIndices = pcwgIndices(3:4);
    descriptors(:,gradIndices) = descriptors(:,gradIndices)*gradWeight;
end
end
