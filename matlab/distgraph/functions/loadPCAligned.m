function pcAligned = loadPCAligned(datasetPath, depthList, sampleinfo, radius)

global dataIndices
sample = struct('frame', sampleinfo(dataIndices.frame), 'pixel', sampleinfo(dataIndices.pixel), ...
                'point', sampleinfo(dataIndices.point), 'normal', sampleinfo(dataIndices.normal), ...
                'praxis', sampleinfo(dataIndices.praxis));
depthFileName = sprintf('%s/%s', datasetPath, depthList{sample.frame,1});
pcSample = loadPointCloud(depthFileName, sample.pixel, radius);
tformSample = transformG2L(sample.point, sample.normal, sample.praxis);
pcAligned = pctransform(pcSample, tformSample);
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
