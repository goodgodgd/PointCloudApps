function showDistances(datasetIndex, radius, numSamples)

global normalDistWeight
datasetPath = workingDir(datasetIndex, radius);
filename = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
samples = load(filename);
samples = samples.samples;
size(samples)
samples(:,1:4);

filename = sprintf('%s/shapeDists_%d.mat', datasetPath, numSamples);
shapeDists = load(filename);
indexPairs = shapeDists.indexPairs;
shapeDists = shapeDists.shapeDists;

% filename = sprintf('%s/descrDists_%d.mat', datasetPath, numSamples);
% descrDists = load(filename);
% descrDists = descrDists.descrDists;

inliers = find(shapeDists(:,1)*1000 < 2.2 & shapeDists(:,2)*10 < 2.7 & shapeDists(:,2)*10 > 1);
inliers = inliers(randperm(length(inliers)));

tfdecision = initDescisions(radius);
szdescisions = size(tfdecision)

for idx = inliers'
    indices = [idx indexPairs(idx,:)];
    pcModel = loadPCAligned(datasetIndex, samples(indexPairs(idx,1),:), radius);
    pcQuery = loadPCAligned(datasetIndex, samples(indexPairs(idx,2),:), radius);
    [tformReg, pcQueryReg] = pcregrigid(pcQuery, pcModel, ...
                                    'Metric', 'pointToPoint', 'InlierRatio', 1);
    drawPointClouds(pcModel, pcQueryReg);
    shapedistance = [shapeDists(idx,1)*1000, shapeDists(idx,2)*10]
    pause
    
    decision = input('give decision: ');
    if isempty(decision)
        continue
    end
    if decision == 4
        break
    end
    newrow = [idx shapedistance decision]
    tfdecision = [tfdecision; newrow];
    

%     try
%         newDistances = distanceBetweenClouds(pcModel, pcQueryReg);
%         newDistances = [newDistances(1)*1000 newDistances(2)*10]
%     catch ME
%         ME.identifier
%         if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
%             warning('%s %s', ME.identifier, ME.message);
%         else
%             rethrow(ME)
%         end
%     end
end

drawDistanceMap(tfdecision);
filename = sprintf('decisions_%d.mat', radius);
save(filename, 'tfdecision');
pause

end

function drawPointClouds(pcModel, pcQuery)
if pcModel.Count > 100
    modelIndices = randperm(pcModel.Count, 100);
else
    modelIndices = 1:pcModel.Count;
end
if pcQuery.Count > 100
    queryIndices = randperm(pcQuery.Count, 100);
else
    queryIndices = 1:pcQuery.Count;
end

% plot point clouds
figure(1);
pcshow(pcModel.Location, [1 0 0]);
hold on
pcshow(pcQuery.Location, [0 0 1]);
quiver3(pcModel.Location(modelIndices,1), pcModel.Location(modelIndices,2), pcModel.Location(modelIndices,3), ...
        pcModel.Normal(modelIndices,1), pcModel.Normal(modelIndices,2), pcModel.Normal(modelIndices,3), 'r');
quiver3(pcQuery.Location(queryIndices,1), pcQuery.Location(queryIndices,2), pcQuery.Location(queryIndices,3), ...
        pcQuery.Normal(queryIndices,1), pcQuery.Normal(queryIndices,2), pcQuery.Normal(queryIndices,3), 'b');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('compare two shapes')
hold off
descNames = {'Model', 'Query'};
legend(descNames)
end

function tfdecision = initDescisions(radius)
filename = sprintf('decisions_%d.mat', radius);
if exist(filename, 'file')
    tfdecision = load(filename);
    tfdecision = tfdecision.tfdecision;
else
    tfdecision = zeros(0,4);
end
end

function drawDistanceMap(tfdecision)
positives = tfdecision(tfdecision(:,4)==1,:);
negatives = tfdecision(tfdecision(:,4)==0,:);
neutral = tfdecision(tfdecision(:,4)==2,:);
figure(2)
plot(positives(:,2), positives(:,3), '.r');
hold on
plot(negatives(:,2), negatives(:,3), '.b');
plot(neutral(:,2), neutral(:,3), '.g');
xlabel('point distance')
ylabel('normal distance')
hold off
end