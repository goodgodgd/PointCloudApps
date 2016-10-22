function D4analyzeResult(datasetIndex, radius, numSamples)

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/distances_%d.mat', datasetPath, numSamples);
dist = load(filename);
dist = dist.distances;
if size(dist.shape,1) < size(dist.shape,2)
    dist.shape = dist.shape';
end
if size(dist.descr,1) < size(dist.descr,2)
    dist.descr = dist.descr';
end
[size(dist.shape) size(dist.descr)]

% scale shape distance
dist.shape = dist.shape*1000;
inclines = zeros(6,1);
fiterrors = zeros(6,1);

figure(1)
subtitles = {'prcv', 'pcwg', 'fpfh', 'shot', 'spin', 'tris'};
for i=1:6
    [inclines(i) fiterrors(i)] = FitLinear(dist.shape, dist.descr(:,i));
    subplot(2, 3, i)
    plot(dist.shape, dist.descr(:,i)/inclines(i), 'b.')
    hold on
    plot([0 max(dist.shape)], [0 max(dist.shape)], 'r-')
    hold off
    title(subtitles{1,i})
end

result = [inclines fiterrors]
k = input('please enter');
end

function [incline error] = FitLinear(shapeDist, descrDist)
incline = pinv(shapeDist)*descrDist;
error = mean(abs(descrDist./shapeDist - incline))/incline;
end