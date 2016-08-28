function D1extractTracks(dsetIndex)

global totalDimension;
addpath('..\funcdist')
initGlobals;
dsetPath = workingDir(dsetIndex)
frameCount = 0;
IDIndex = 1;
descBegin = 9;

sgtrack = struct('ID', 0, 'descriptors', zeros(0,totalDimension));
tracks = repmat(sgtrack, 10000, 1);

while(frameCount<2000)
    frameCount=frameCount+1;
    filename = sprintf('%s/DDS_%05d.txt', dsetPath, frameCount);
    if(exist(filename, 'file')==0)
        break;
    end
    data = load(filename);
    
    for di=1:size(data,1)
        dataID = data(di,IDIndex);
        if(dataID > length(tracks))
            'out of track indices'
            continue;
        end
        neoline = size(tracks(dataID).descriptors,1) + 1;
        if tracks(dataID).ID == 0
            tracks(dataID).ID = dataID;
        elseif tracks(dataID).ID ~= dataID
        	error('inconsistent ID')
        	continue;
        end
        
        tracks(dataID).descriptors = [tracks(dataID).descriptors; data(di,descBegin:descBegin+totalDimension-1)];
    end
    sprintf('frame %d is added', frameCount)
end

tracks = trimLowCountTracks(tracks, 20);
filename = sprintf('%s/tracks.mat', dsetPath);
save(filename, 'tracks')
