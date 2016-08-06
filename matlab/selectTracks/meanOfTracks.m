function trackMean = meanOfTracks(tracks)

global totalDimension;

trackMean = zeros(length(tracks), totalDimension);
for i=1:length(tracks)
    trackMean(i,:) = mean(tracks(i).descriptors,1);
end
