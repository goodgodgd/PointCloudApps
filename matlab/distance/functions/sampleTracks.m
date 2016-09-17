% function sampleTracks(dsetIndex)
clc
clear
dsetIndex = 1;

addpath('../utils')
initGlobals;
dsetPath = workingDir(dsetIndex);
trackFileName = sprintf('%s/tracks.mat', dsetPath);
if exist(trackFileName, 'file')==0
    error('track file does not exist: %s', trackFileName)
else
    srcTracks = load(trackFileName);
end

srcTracks = srcTracks.tracks;
highCurvCount = countHighCurvs(srcTracks)

numClusters = 100;
bsTracks = balanceSamples(srcTracks, numClusters);
tracks = sampleReprstt(bsTracks, numClusters);

sampleFileName = sprintf('%s/sample.mat', dsetPath);
save(sampleFileName, 'tracks');
'samples saved'
