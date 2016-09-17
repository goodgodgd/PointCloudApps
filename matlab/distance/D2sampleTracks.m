function D2sampleTracks(dsetIndex, radius, numTracks)
% clc
% clear
% dsetIndex = 1;

global totalDimension; global eachDescIndices;

addpath('../funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex, radius);
trackFileName = sprintf('%s/tracks.mat', dsetPath);
if exist(trackFileName, 'file')==0
    error('track file does not exist: %s', trackFileName)
else
    srcTracks = load(trackFileName);
end

srcTracks = srcTracks.tracks;
highCurvCount = countHighCurvs(srcTracks)
occurrences = countOccurrences(srcTracks)

bsTracks = balanceSamples(srcTracks, numTracks);
tracks = sampleReprstt(bsTracks, numTracks);

occurrences = countOccurrences(tracks)

sampleFileName = sprintf('%s/sample_%d.mat', dsetPath, numTracks);
save(sampleFileName, 'tracks');
'samples saved'
