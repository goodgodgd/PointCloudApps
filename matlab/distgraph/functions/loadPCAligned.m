function [pcAligned, imgpxs] = loadPCAligned(datasetPath, sampleinfo)

global dataIndices
sample = struct('frame', sampleinfo(dataIndices.frame), 'pixel', sampleinfo(dataIndices.pixel), ...
                'point', sampleinfo(dataIndices.point), 'normal', sampleinfo(dataIndices.normal), ...
                'praxis', sampleinfo(dataIndices.praxis));
sample.pixel = sample.pixel + [1 1];

depthList = getDepthList(datasetPath);
depthFileName = depthList{sample.frame};
if nargout==2
    [pcSample, imgpxs] = loadPointCloud(depthFileName, sample.pixel);
else
    pcSample = loadPointCloud(depthFileName, sample.pixel);
end
checkValidity(sample, pcSample);

tformSample = transformG2L(sample.point, sample.normal, sample.praxis);
pcAligned = pctransform(pcSample, tformSample);
end


function checkValidity(sample, pcSample)

if norm(sample.point - pcSample.Location(1,:)) < 0.01
    sample.point = pcSample.Location(1,:);
else
    sample
    pcSample.Location(1,:)
    distance = norm(sample.point - pcSample.Location(1,:))
    ME = MException('shapeDistance:loadPCAligned', 'center point is not correct %d (%d, %d)', ...
                    sample.frame, sample.pixel(1), sample.pixel(2));
    throw(ME)
end
end

function output = getDepthList(datasetPath)

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
output = depthList;
end


function tform = transformG2L(center, firstAxis, secondAxis)

firstAxis = firstAxis/norm(firstAxis);
lastAxis = cross(firstAxis, secondAxis);
lastAxis = lastAxis/norm(lastAxis);
secondAxis = cross(lastAxis, firstAxis);
secondAxis = secondAxis/norm(secondAxis);

% global to local rotation
g2lrot = [firstAxis; secondAxis; lastAxis];
% local to global rotation
l2grot = g2lrot';
% local to global transformation
tfmat = [l2grot reshape(center,3,1); 0 0 0 1];
% global to local transformation
tfmat = inv(tfmat);
tform = affine3d(tfmat');
end
