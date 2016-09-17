function count = countHighCurvs(tracks)

global eachDescIndices;
pcwgIdcs = eachDescIndices(1);
trackMeans = meanOfTracks(tracks);
count=0;
for i=1:length(tracks)
    if trackMeans(i,pcwgIdcs(1)) > 3 || trackMeans(i,pcwgIdcs(2)) > 3
        count=count+1;
    end
end
