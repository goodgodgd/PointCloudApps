function D1extractTracks(dsetIndex, radius)

global totalDimension;
addpath('functions')
initGlobals;
dsetPath = workingDir(dsetIndex, radius)
frameCount = 0;
IdCols = 1;
pixelCols = 4:5;
descBegin = 9;
descCols = descBegin:descBegin+totalDimension-1;

sgtrack = struct('ID', 0, 'frames', zeros(0,1), 'pixels', zeros(0,2),...
                'descriptors', zeros(0,totalDimension));
tracks = repmat(sgtrack, 10000, 1);

while(frameCount<2000)
    frameCount=frameCount+1;
    filename = sprintf('%s/DDS_%05d.txt', dsetPath, frameCount);
    if(exist(filename, 'file')==0)
        break;
    end
    data = load(filename);
    
    for di=1:size(data,1)
        dataID = data(di,IdCols);
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
        
        tracks(dataID).frames(end+1) = frameCount;
        tracks(dataID).pixels(end+1,:) = data(di,pixelCols);
        tracks(dataID).descriptors(end+1,:) = data(di,descCols);
    end
    sprintf('frame %d is added', frameCount)
end

tracks = trimLowCountTracks(tracks, 20);
filename = sprintf('%s/tracks.mat', dsetPath);
save(filename, 'tracks')
