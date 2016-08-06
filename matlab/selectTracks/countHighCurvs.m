function count = countHighCurvs(tracks)

trackMeans = meanOfTracks(tracks);
count=0;
for i=1:length(tracks)
    if trackMeans(i,1) > 3 || trackMeans(i,2) > 3
        count=count+1;
    end
end
