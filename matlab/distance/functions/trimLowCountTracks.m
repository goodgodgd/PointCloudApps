function tracks = trimLowCountTracks(tracks, countLowLimit)

validIndices = zeros(0,0);
for i=1:length(tracks)
    if length(tracks(i).frames) > countLowLimit
        validIndices(end+1) = i;
    end
end

tracks = tracks(validIndices);
