function testInvariance()
clc
clear

global radius
addpath('../functions')
initGlobals

radii = [5];
numDsets = dataPath();
dsetIndices = 1:9;
scale = 4;
loopIndices = createLoopIndices(radii, dsetIndices)
loopLength = size(loopIndices,1);
top1_accuracy = zeros(loopLength,10);
top5_accuracy = zeros(loopLength,10);

for li = 1:loopLength
    dataIndex = loopIndices(li,2);
    radius = loopIndices(li,3);
    datasetPath = dataPath(dataIndex, radius)
    setCameraParams(datasetPath, 2)

%     querySet = readQuerySet(datasetPath, scale);
%     [referSet] = readReferenceSet(datasetPath);
%     filterValidSamples(datasetPath, referSet, querySet, scale);
%     
%     [referSet, querySet, referIndices] = loadValidSamples(datasetPath, scale);
%     calcShapeDists(datasetPath, referSet, querySet, scale);
%     good
%     computeQueryBRAND(datasetPath, querySet, scale);
%     calcDescDists(datasetPath, referSet, referIndices, querySet, scale);
    
    [top1, top5] = analyzeResult(datasetPath, scale)
    top1_accuracy(li,:) = [dataIndex, radius, top1];
    top5_accuracy(li,:) = [dataIndex, radius, top5];
end

% rank PCWG and append information on accuracy
top1_accuracy = rankAccuracy(top1_accuracy);
top5_accuracy = rankAccuracy(top5_accuracy);
filename = sprintf('../../../../top1_accuracy_sc%d.mat', scale);
save(filename, 'top1_accuracy')
filename = sprintf('../../../../top5_accuracy_sc%d.mat', scale);
save(filename, 'top5_accuracy')
end

function loopIndices = createLoopIndices(radii, dsetIndices)
loopIndices = zeros(length(radii)*length(dsetIndices), 3);
loopcnt = 0;
for radius = radii
    for dataIndex = dsetIndices
        loopcnt = loopcnt+1;
        loopIndices(loopcnt,:) = [loopcnt dataIndex radius];
    end
end
end

function querySet = readQuerySet(datasetPath, scale)
filename = [datasetPath, sprintf('/sampleScale%d.txt', scale)];
querySet = load(filename);
querySet = normalizeAllDescs(querySet);
end

function outdata = normalizeAllDescs(inpdata)
global dataIndices
outdata = inpdata;

% FPFH
outdata(:,dataIndices.descrs(3)) = normalizeDescs(outdata(:,dataIndices.descrs(3)));
% SHOT
outdata(:,dataIndices.descrs(4)) = normalizeDescs(outdata(:,dataIndices.descrs(4)));
% SPIN
outdata(:,dataIndices.descrs(5)) = normalizeDescs(outdata(:,dataIndices.descrs(5)));
% TRIS
indices = dataIndices.descrs(5) + length(dataIndices.descrs(5));
outdata(:,indices) = normalizeDescs(outdata(:,indices));
indices = dataIndices.descrs(5) + length(dataIndices.descrs(5))*2;
outdata(:,indices) = normalizeDescs(outdata(:,indices));
end

function data = normalizeDescs(data)
sumDescs = sum(data,2);
for i=1:size(data,1)
    data(i,:) = data(i,:)/sumDescs(i);
end    
end

function [referSet] = readReferenceSet(datasetPath)
global numSamples
fileName = sprintf('%s/sample_%d.mat', datasetPath, numSamples);
if exist(fileName, 'file')==0
    error('data file does not exist: %s', fileName)
else
    samples = load(fileName);
end
referSet = samples.samplesRefer;
end


function filterValidSamples(datasetPath, referSet, querySet, queryScale)
% before_data = [referSet(1:size(querySet,1),[1 2 4]) querySet(:,[1 2 4])]
validReferIndices = querySet(:,1) + 1;   % query was written in zero-base index
validQueryIndices = 1:size(querySet,1);
% sample index to frame index
querySet(:,1) = referSet(validReferIndices,1);

for qi=validQueryIndices
    ri = validReferIndices(qi);
    if isValidQueryData(querySet(qi,:), referSet(ri,:))==0
        validReferIndices(qi) = 0;
        validQueryIndices(qi) = 0;
    end

    try
        setCameraParams(datasetPath, 2);
        loadPCAligned(datasetPath, referSet(ri,:));
        setCameraParams(datasetPath, queryScale);
        loadPCAligned(datasetPath, querySet(qi,:));
    catch ME
        ME.identifier
        if strncmpi(ME.identifier, 'shapeDistance', length('shapeDistance'))
            warning('%s %s', ME.identifier, ME.message);
            validReferIndices(qi) = 0;
            validQueryIndices(qi) = 0;
        else
            rethrow(ME)
        end
    end
end
% set default
setCameraParams(datasetPath, 2);

validReferIndices = validReferIndices(validReferIndices>0);
validQueryIndices = validQueryIndices(validQueryIndices>0);
referSet = referSet(validReferIndices,:);
querySet = querySet(validQueryIndices,:);
filtered_sample_size = size(referSet,1)

filename = sprintf('%s/sampleScale%d.mat', datasetPath, queryScale);
save(filename, 'referSet', 'querySet', 'validReferIndices');
end

function validity = isValidQueryData(querySample, referSample)
global dataIndices
pcindices = dataIndices.descrs(1);
validity = 1;
if querySample(pcindices(1))==0
    validity = 0;
%     pcindices
%     query_pcwg_descriptor = querySample(pcindices)
elseif sum(abs(querySample(dataIndices.point) - referSample(dataIndices.point))) > 0.03
    validity = 0;
%     different_point = [querySample(dataIndices.point), referSample(dataIndices.point)]
elseif abs(dot(querySample(dataIndices.normal), referSample(dataIndices.normal))) < cos(deg2rad(20))
    validity = 0;
%     different_normal = [querySample(dataIndices.normal), referSample(dataIndices.normal), ...
%         abs(dot(querySample(dataIndices.normal), referSample(dataIndices.normal))), cos(deg2rad(20))]
end
end

function [referSet, querySet, referIndices] = loadValidSamples(datasetPath, scale)
filename = sprintf('%s/sampleScale%d.mat', datasetPath, scale);
sampleData = load(filename);
referSet = sampleData.referSet;
querySet = sampleData.querySet;
referIndices = sampleData.validReferIndices;
filtered_sample_size = size(referSet,1)
end


function calcShapeDists(datasetPath, referSamples, querySamples, queryScale)
referSize = size(referSamples,1);
querySize = size(querySamples,1);
shapeDists = ones(referSize, querySize, 2);
for ri=1:referSize
    shapeDists(ri,ri,:) = [0, 0];
end

filename = sprintf('%s/shapeDistsScale%d.mat', datasetPath, queryScale);
save(filename, 'shapeDists');
'shape distances saved'
end


function computeQueryBRAND(datasetPath, querySamples, scale)
'compute query sample BRAND'
global brandLength
setCameraParams(datasetPath, scale);
querySize = size(querySamples,1);
queryBrand = zeros(querySize, brandLength);

tic
for i=1:querySize
    queryBrand(i,:) = BRAND(datasetPath, querySamples(i,:));
end
toc

sampleFileName = sprintf('%s/BRAND_Scale%d.mat', datasetPath, scale);
save(sampleFileName, 'queryBrand');
end


function calcDescDists(datasetPath, referSamples, referIndices, querySamples, queryScale)
global dataIndices gradWeight numDescTypes numSamples

filename = sprintf('%s/BRAND_%d.mat', datasetPath, numSamples);
BRAND = load(filename);
brandRefer = BRAND.referBrand;
brandRefer = brandRefer(referIndices,:);

filename = sprintf('%s/BRAND_Scale%d.mat', datasetPath, queryScale);
BRAND = load(filename);
brandQuery = BRAND.queryBrand;

referSize = size(referSamples,1);
querySize = size(querySamples,1);
sample_sizes = [referSize size(brandRefer,1) querySize size(brandQuery,1)]
assert(referSize==size(brandRefer,1) && querySize==size(brandQuery,1));
descrDists = zeros(referSize, querySize, numDescTypes);

% apply gradient weight
gradIndices = dataIndices.descrs(2);
gradIndices = gradIndices(3:4);
referSamples(:,gradIndices) = referSamples(:,gradIndices)*gradWeight;
querySamples(:,gradIndices) = querySamples(:,gradIndices)*gradWeight;

for ri=1:referSize
    for qi=1:querySize
        descrDists(ri,qi,1:6) = descrDistance(referSamples(ri,:), querySamples(qi,:));
        descrDists(ri,qi,7) = sum(abs(brandRefer(ri,:)-brandQuery(qi,:)));
    end
end

filename = sprintf('%s/descrDistsScale%d.mat', datasetPath, queryScale);
save(filename, 'descrDists');
'descriptor distances saved'
end

function descrDist = descrDistance(referSamples, querySamples)
global dataIndices
descrDist = zeros(1,6);
sampleDiffAbs = abs(referSamples - querySamples);
for dtype=1:6
    descrDist(dtype) = sum(sampleDiffAbs(dataIndices.descrs(dtype)));
end
end


function [top1_acc, top5_acc] = analyzeResult(datasetPath, scale)
'Analyze result'
global normalDistWeight radius
filename = sprintf('%s/shapeDistsScale%d.mat', datasetPath, scale);
shapeDists = load(filename);
shapeDists = shapeDists.shapeDists;
shapeDists = shapeDists(:,:,1) + shapeDists(:,:,2)*normalDistWeight;
sdScale = 1000;
shapeDists = shapeDists*sdScale;
size_shapeDists = size(shapeDists)

filename = sprintf('%s/descrDistsScale%d.mat', datasetPath, scale);
descrDists = load(filename);
descrDists = descrDists.descrDists;

top1_acc = top1Accuracy(shapeDists, descrDists, radius);
top5_acc = top5Accuracy(shapeDists, descrDists, radius);
end

function accuracy = top1Accuracy(shapeDists, descrDists, radius_cm)
[cshapeValue, cshapeIndex] = min(shapeDists);
withinRadius = cshapeValue<radius_cm*5;
cshapeIndex = cshapeIndex(withinRadius);
accuracy = zeros(1,8);
accuracy(8) = sum(withinRadius);

for i=1:7
    [~, cdescrIndex] = min(descrDists(:,:,i));
    cdescrIndex = cdescrIndex(withinRadius);
    accuracy(i) = mean(cshapeIndex==cdescrIndex);
end
end

function accuracy = top5Accuracy(shapeDists, descrDists, radius_cm)
[cshapeValue, cshapeIndex] = min(shapeDists);
withinRadius = cshapeValue<radius_cm*10;
cshapeIndex = cshapeIndex(withinRadius);
accuracy = zeros(1,8);
accuracy(8) = sum(withinRadius);

for i=1:7
    [sortedDists, sdescrIndices] = sort(descrDists(:,:,i), 1);
    if i==7
        sortedDists = sortedDists(1:8,withinRadius)';
    end
    sdescrIndices = sdescrIndices(1:5, withinRadius);
    top5tf = sum(repmat(cshapeIndex, 5, 1)==sdescrIndices, 1);
    accuracy(i) = mean(top5tf);
end
end

function accuracy = rankAccuracy(accuracy)
descCols = 3:9;
pcwgIndex = 2;
numDescTypes = length(descCols);
[~, sorted_indices] = sort(accuracy(:,descCols), 2, 'descend');
pcwg_indices = find(sorted_indices'==pcwgIndex) - 1;
pcwg_raking = mod(pcwg_indices, numDescTypes) + 1;
accuracy = [accuracy pcwg_raking]
mean_accuracy = mean(accuracy(:,descCols))
end

