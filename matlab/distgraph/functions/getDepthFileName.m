function depthFileName = getDepthFileName(datasetPath, frameIndex)

persistent depthList dsetPathBef
if isempty(dsetPathBef)
    dsetPathBef = '.';
end

if isempty(depthList) || strcmp(datasetPath, dsetPathBef)==0
    filename = sprintf('%s/depthList.txt', datasetPath);
    fid = fopen(filename);
    depthList = textscan(fid,'%s','Delimiter','\n');
    depthList = depthList{1,1};
    dsetPathBef = datasetPath;
end

depthFileName = depthList{frameIndex};
end

