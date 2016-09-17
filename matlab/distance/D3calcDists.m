function D3calcDists(dsetIndex, radius, numTracks)
% clc
% clear
% dsetIndex = 1;

global totalDimension; global eachDescIndices;

addpath('../funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex, radius);
trackFileName = sprintf('%s/sample_%d.mat', dsetPath, numTracks);
if exist(trackFileName, 'file')==0
    error('track file does not exist: %s', trackFileName)
else
    tracks = load(trackFileName);
end
tracks = tracks.tracks;
if size(tracks(1).descriptors,2)~=totalDimension
    error('track descriptor dimension is not appropriate')
end

NT = length(tracks)
Z = zeros(NT);
BD = struct('prcv', Z, 'pcwg', Z, 'spin', Z, 'fpfh', Z, 'shot', Z, 'tris', Z);

for i=1:NT-1
    for k=i+1:NT
        BD.prcv(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(1)), ...
                                    tracks(k).descriptors(:,eachDescIndices(1)));
        BD.pcwg(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(2)), ...
                                    tracks(k).descriptors(:,eachDescIndices(2)));
        BD.spin(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(3)), ...
                                    tracks(k).descriptors(:,eachDescIndices(3)));
        BD.fpfh(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(4)), ...
                                    tracks(k).descriptors(:,eachDescIndices(4)));
        BD.shot(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(5)), ...
                                    tracks(k).descriptors(:,eachDescIndices(5)));
        BD.tris(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(6)), ...
                                    tracks(k).descriptors(:,eachDescIndices(6)));
    end
    sprintf('row %d done', i)
end

sampleFileName = sprintf('%s/BD_%d.mat', dsetPath, numTracks);
save(sampleFileName, 'BD');

BDv.pcwg = BD.pcwg(BD.pcwg~=0);
BDv.spin = BD.spin(BD.spin~=0);
BDv.fpfh = BD.fpfh(BD.fpfh~=0);
BDv.shot = BD.shot(BD.shot~=0);
BDv.tris = BD.tris(BD.tris~=0);
pcwg = mean(BDv.pcwg)
spin = mean(BDv.spin)
fpfh = mean(BDv.fpfh)
shot = mean(BDv.shot)
tris = mean(BDv.tris)

return

pcwgSame = 0;
spinSame = 0;
fpfhSame = 0;
shotSame = 0;
for i=1:2:NT
    pcwgSame = pcwgSame + BD.pcwg(i,i+1)/NT;
    spinSame = spinSame + BD.spin(i,i+1)/NT;
    fpfhSame = fpfhSame + BD.fpfh(i,i+1)/NT;
    shotSame = shotSame + BD.shot(i,i+1)/NT;
end

pcwgSame
spinSame
fpfhSame
shotSame

pcwgAdj = pcwg / pcwgSame
spinAdj = spin / spinSame
fpfhAdj = fpfh / fpfhSame
shotAdj = shot / shotSame
