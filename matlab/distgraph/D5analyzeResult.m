function D5analyzeResult(datasetIndex, radius, numSamples)

global distThreshForGradWeight normalDistWeight

datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
% mix point distance and normal angle difference with ratio of 200
shapeDists = shapeDists(:,1) + shapeDists(:,2)*normalDistWeight;

filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
descrDists = load(filename);
descrDists = descrDists.descrDists;

if size(shapeDists,1) ~= size(descrDists,1)
    error('inconsistent data size: shape=%d, descr=%d', size(shapeDists,1), size(descrDists,1));
end

numTrueSamples = [sum(shapeDists < distThreshForGradWeight(radius)) length(shapeDists)]

% scale shape distance
sdScale = 1000;
shapeDists = shapeDists*sdScale;
zr = zeros(20,2);
perfmcs = containers.Map({1, 2, 3, 4, 5, 6, 7}, {zr, zr, zr, zr, zr, zr, zr});
for i=1:7
    perfmcs(i) = calc4Performs(shapeDists, descrDists(:,i), distThreshForGradWeight(radius)*sdScale);
end

descNames = {'prcv', 'pcwg', 'fpfh', 'shot', 'spin', 'tris', 'pcwgOpt'};
drawScatterPlots(shapeDists, descrDists, descNames, distThreshForGradWeight(radius)*sdScale)
drawROCCurves(perfmcs, descNames)
pause

end

function metrics = calc4Performs(shapeDists, descrDists, sdistThresh)
metlen = 100;
metrics = zeros(metlen,2);
ddistMax = max(descrDists);
ddistThresh = 0:ddistMax/metlen:ddistMax;
tsplit = zeros(metlen,4);
for i=1:metlen
    TP = sum(shapeDists<sdistThresh & descrDists<ddistThresh(i));
    FN = sum(shapeDists<sdistThresh & descrDists>ddistThresh(i));
    FP = sum(shapeDists>sdistThresh & descrDists<ddistThresh(i));
    TN = sum(shapeDists>sdistThresh & descrDists>ddistThresh(i));
    metrics(i,1) = TP/(TP+FN);
    metrics(i,2) = FP/(FP+TN);
    tsplit(i,:) = [TP FN FP TN];
end
end

function drawScatterPlots(shapeDists, descrDists, descNames, sdistThresh)
figure(1)
for i=1:7
    subplot(2,4,i)
    plot(shapeDists, descrDists(:,i), 'b.')
    hold on
    plot([sdistThresh sdistThresh], [0 max(descrDists(:,i))], 'r-')
    hold off
    title(descNames{1,i})
end
end

function drawROCCurves(perfmcs, descNames)
figure(2)
colors = lines(7);
% indices = [1 2 7];
indices = 1:7;
for i=indices
    metrics = perfmcs(i);
    plot(metrics(:,2), metrics(:,1), '-', 'Color', colors(i,:))
    if i==1
        hold on
    end
end
legend(descNames{1,indices})
hold off
end




















