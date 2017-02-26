function brandout = BRAND(datasetPath, sample, print)
global radius printout
ori_radius = radius;
radius = 10;
if nargin==3
    printout = print;
else
    printout = 0;
end

pattern = readPattern();
[pointCloud, imgpts] = loadPCAligned(datasetPath, sample);
pattern = transformPattern(pattern, pointCloud.Location(1,:), pointCloud.Normal(1,:));
pattIndices = patternToPointIndices(pattern, imgpts, pointCloud);
brandout = computeBRAND(pointCloud, pattIndices);

radius = ori_radius;
if printout
    drawPointCloud(pointCloud,0,1)
    drawCorrepondences(pointCloud, pattIndices);
    pause
end
end


function pattern = readPattern()

global brandLength
persistent patternout
if ~isempty(patternout)
    'return loaded BRAND pattern';
    pattern = patternout;
    return
end

if exist('BRANDpattern.mat', 'file')
    'read BRAND pattern';
    pattern = load('BRANDpattern.mat');
    pattern = pattern.pattern;
else
    'create BRAND pattern';
    brandLenx2 = brandLength*2;
    stdDev = 48/5;
    pattern = ones(brandLenx2,2)*1000;
    for i=1:brandLenx2
        while norm(pattern(i,:)) > 24
            pattern(i,:) = randn(2,1)*stdDev;
        end
    end
    pattern = round(pattern);
    save('BRANDpattern.mat', 'pattern');
end

patternout = pattern;
end


function pattern = transformPattern(pattern, point, normal)
global brandLength printout
scale = max(0.2, (3.8 - 0.4*max(2,point(1)))/3);
polar = acos(dot(point/norm(point), normal/norm(normal)));
azimuth = atan2(-normal(2), -normal(3));
if printout
    angles = [polar/pi*180 cos(polar) azimuth/pi*180]
end
rotation = [cos(azimuth), -sin(azimuth); sin(azimuth), cos(azimuth)];
pattern = scale*pattern;
pattern(:,2) = cos(polar)*pattern(:,2); % scale short axis
pattern = pattern*rotation';
pattern = round(pattern);

assert(size(pattern,1)==brandLength*2);
end


function pattIndices = patternToPointIndices(pattern, imgpts, pointCloud)
global imgWidth imgHeight brandLength printout
trnPattern = pattern + imgpts(1,:);
outliers = (trnPattern(:,1)<1) | (trnPattern(:,1)>imgWidth) ...
            | (trnPattern(:,2)<1) | (trnPattern(:,2)>imgHeight);
pattern_outlier_count = sum(outliers);

trnPattern(outliers,:) = 0;
patternPixelIndices = trnPattern(:,1)*imgWidth + trnPattern(:,2);
imagePixelIndices = imgpts(:,1)*imgWidth + imgpts(:,2);

patternLen = length(patternPixelIndices);
pattIndices = zeros(patternLen,1);
assert(patternLen==brandLength*2);
count=0;

for pi = 1:patternLen
    if patternPixelIndices(pi)==0
        'outlier pixel';
        continue;
    end
    ptindex = find(imagePixelIndices==patternPixelIndices(pi));
    if(isempty(ptindex))
        'no image pixel corresponding to pattern';
        continue
    end
    pattIndices(pi) = ptindex(1);
    count = count+1;
end
if printout
    valid_pattern_count = count;
end
end


function brandout = computeBRAND(pointCloud, pattIndices)
global brandLength printout
patternPairs = reshape(pattIndices,[],2);
assert(brandLength==size(patternPairs,1));
brandout = zeros(brandLength,1);
for i=1:brandLength
    brandout(i) = binaryCheck(pointCloud, patternPairs(i,:));
end
if printout
    brand_sum = sum(brandout);
end
end


function result = binaryCheck(pointCloud, pairIndices)
result = 0;
if pairIndices(1)==0 || pairIndices(2)==0
    return
end
normalDispThresh = cos(pi/4);
normalDisp = dot(pointCloud.Normal(pairIndices(1),:), pointCloud.Normal(pairIndices(2),:));
if normalDisp > normalDispThresh
    return
end

pointDiff = pointCloud.Location(pairIndices(1),:) - pointCloud.Location(pairIndices(2),:);
normalDiff = pointCloud.Normal(pairIndices(1),:) - pointCloud.Normal(pairIndices(2),:);
if dot(pointDiff, normalDiff) < 0   % concave
    return
end
result = 1;
end


function drawCorrepondences(pointCloud, pattIndices)
global brandLength
patternPairs = reshape(pattIndices,[],2);
hold on
for i=1:brandLength
    pair = patternPairs(i,:);
    if pair(1)==0 || pair(2)==0
        continue;
    end
    plot3(pointCloud.Location(pair,1), pointCloud.Location(pair,2), pointCloud.Location(pair,3), 'b-')
end
hold off
end









