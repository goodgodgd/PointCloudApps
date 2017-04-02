function [pcAligned, imgpxs] = loadPCAligned(datasetPath, sampleinfo)

global dataIndices
sample = struct('frame', sampleinfo(dataIndices.frame), 'pixel', sampleinfo(dataIndices.pixel), ...
                'point', sampleinfo(dataIndices.point), 'normal', sampleinfo(dataIndices.normal), ...
                'praxis', sampleinfo(dataIndices.praxis));
sample.pixel = sample.pixel + [1 1];

depthFileName = getDepthFileName(datasetPath, sample.frame);
if nargout==2
    [pcSample, imgpxs] = loadPointCloud(depthFileName, sample.pixel);
else
    pcSample = loadPointCloud(depthFileName, sample.pixel);
end
checkValidity(datasetPath, sample, pcSample);

tformSample = transformG2L(sample.point, sample.normal, sample.praxis);
pcAligned = pctransform(pcSample, tformSample);
end


function checkValidity(datasetPath, sample, pcSample)

distThresh = 0.01;
if ~isempty(strfind(datasetPath, '_gn1')) || ~isempty(strfind(datasetPath, '_gn2'))
    distThresh = 0.015;
elseif ~isempty(strfind(datasetPath, '_gn3'))
    distThresh = 0.02;
end

if norm(sample.point - pcSample.Location(1,:)) < distThresh
    sample.point = pcSample.Location(1,:);
else
    distance = norm(sample.point - pcSample.Location(1,:));
    center_point_distance = [sample.point; pcSample.Location(1,:); distance,0,0]
    ME = MException('shapeDistance:loadPCAligned', 'center point is not correct %d (%d, %d)', ...
                    sample.frame, sample.pixel(1), sample.pixel(2));
    throw(ME)
end
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
