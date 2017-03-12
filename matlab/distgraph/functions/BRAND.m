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
angle = computeOrientation(datasetPath, sample);
pattern = transformPattern(pattern, pointCloud.Location(1,:), angle);
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
    stdDev = 48/5;
    pattern = ones(brandLength,2)*1000;
    for i=1:brandLength
        while norm(pattern(i,:)) > 24
            pattern(i,:) = randn(2,1)*stdDev;
        end
    end
    pattern = [pattern; -pattern];
    pattern = round(pattern);
    save('BRANDpattern.mat', 'pattern');
end

patternout = pattern;
end


function pattern = transformPattern(pattern, point, angle)
global brandLength printout
scale = max(0.2, (3.8 - 0.4*max(2,point(1)))/3);
rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
if printout
    brand_scale_angle = [scale, angle, angle/pi*180]
end
pattern = scale*pattern*rotation';
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


function angle = computeOrientation(datasetPath, sampleinfo)

global radius dataIndices printout
sample = struct('frame', sampleinfo(dataIndices.frame), 'pixel', sampleinfo(dataIndices.pixel), ...
                'point', sampleinfo(dataIndices.point), 'normal', sampleinfo(dataIndices.normal), ...
                'praxis', sampleinfo(dataIndices.praxis));
sample.pixel = sample.pixel + [1 1];

depthFileName = getDepthFileName(datasetPath, sample.frame);
[depthmap, boundbox] = loadDepthMapScaled(depthFileName, sample.pixel, radius);

intImage = integralImage(depthmap);
% Construct Haar-like wavelet filters. Use the dot notation to find the vertical filter from the horizontal filter.
vertH = integralKernel([1 1 4 3; 1 4 4 3],[-1, 1]);
horiH = vertH.';
% Compute the filter responses.
horiResponse = integralFilter(intImage,horiH);
vertResponse = integralFilter(intImage,vertH);

ctpx = round([size(depthmap,2)/2+0.5, size(depthmap,1)/2+0.5]);
horiDiff = horiResponse(ctpx(2), ctpx(1));
vertDiff = vertResponse(ctpx(2), ctpx(1));
angle = atan2(vertDiff, horiDiff);

if printout
brand_orientation = [horiDiff, vertDiff, angle, angle/pi*180]
figure(2)
subplot(2,2,1)
imshow(depthmap, [])
subplot(2,2,3)
imshow(horiResponse, [])
subplot(2,2,4)
imshow(vertResponse, [])
min_max_diff = [min(min(horiResponse)), max(max(horiResponse)), ...
                min(min(vertResponse)), max(max(vertResponse))]
end
end






