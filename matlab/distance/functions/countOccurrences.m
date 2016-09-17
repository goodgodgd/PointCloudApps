function [ occurrences ] = countOccurrences( tracks )

trackCounts = zeros(length(tracks),1);
for i=1:length(tracks)
    trackCounts(i) = size(tracks(i).descriptors, 1);
end
occurList = unique(trackCounts);
occurrences = [occurList, histc(trackCounts, occurList)];

