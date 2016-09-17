clc
clear

radii = [4, 6];
numTracks = 50;

for radius = radii
    for i=1:3
        D1extractTracks(i, radius);
%         D2sampleTracks(i, radius, numTracks);
%         D3calcDists(i, radius, numTracks);
    end
end
