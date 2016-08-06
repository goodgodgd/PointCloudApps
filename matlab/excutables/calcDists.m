clc
clear

global IDIndex; global frameIndex; global totalDimension; global eachDescIndices;

addpath('../funcdist')
initGlobals;
dsetPath = workingDir(1);
trackFileName = sprintf('%s/sample.mat', dsetPath);
if exist(trackFileName, 'file')==0
    error('track file does not exist: %s', trackFileName)
else
    tracks = load(trackFileName);
end
tracks = tracks.tracks;
if size(tracks(1).descriptors,2)~=totalDimension
    error('track descriptor dimension is not appropriate')
end

N = length(tracks)
Z = zeros(N);
BD = struct('cwg', Z, 'spin', Z, 'fpfh', Z, 'shot', Z);

for i=1:N-1
    for k=i+1:N
        BD.cwg(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(1)), ...
                                    tracks(k).descriptors(:,eachDescIndices(1)));
        BD.spin(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(2)), ...
                                    tracks(k).descriptors(:,eachDescIndices(2)));
        BD.fpfh(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(3)), ...
                                    tracks(k).descriptors(:,eachDescIndices(3)));
        BD.shot(i,k) = gBD(tracks(i).descriptors(:,eachDescIndices(3)), ...
                                    tracks(k).descriptors(:,eachDescIndices(3)), 0.02);
    end
    sprintf('row %d done', i)
end

BDv.cwg = BD.cwg(BD.cwg~=0);
BDv.spin = BD.spin(BD.spin~=0);
BDv.fpfh = BD.fpfh(BD.fpfh~=0);
BDv.shot = BD.shot(BD.shot~=0);
cwg  = mean(BDv.cwg)
spin = mean(BDv.spin)
fpfh = mean(BDv.fpfh)
shot = mean(BDv.shot)

return

cwgSame = 0;
spinSame = 0;
fpfhSame = 0;
shotSame = 0;
for i=1:2:N
    cwgSame = cwgSame + BD.cwg(i,i+1)/N;
    spinSame = spinSame + BD.spin(i,i+1)/N;
    fpfhSame = fpfhSame + BD.fpfh(i,i+1)/N;
    shotSame = shotSame + BD.shot(i,i+1)/N;
end

cwgSame
spinSame
fpfhSame
shotSame

cwgAdj = cwg / cwgSame
spinAdj = spin / spinSame
fpfhAdj = fpfh / fpfhSame
shotAdj = shot / shotSame
