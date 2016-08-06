function tracks = trimLowCountTracks(tracks, countLowLimit)

curIdx = 0;
for i=1:length(tracks)
    if size(tracks(i).descriptors,1) > countLowLimit
        curIdx = curIdx+1;
        tracks(curIdx) = tracks(i);
    end
end

tracks = tracks(1:curIdx);
