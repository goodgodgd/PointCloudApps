% function sampleTracks(dsetIndex)
clc
clear
dsetIndex = 1;

addpath('../funcdist')
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
occurrences = countOccurrences(srcTracks)

numClusters = 100;
bsTracks = balanceSamples(srcTracks, numClusters);
tracks = sampleReprstt(bsTracks, numClusters);

occurrences = countOccurrences(tracks)

sampleFileName = sprintf('%s/sample.mat', dsetPath);
save(sampleFileName, 'tracks');
'samples saved'
