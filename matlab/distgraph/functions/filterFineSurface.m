function dstData = filterFineSurface(srcData, datasetIndex, radius, numSamples)

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/depthList.txt', datasetPath);
fid = fopen(filename);
depthList = textscan(fid,'%s','Delimiter','\n');
depthList = depthList{1,1};
depthList = depthList(2:end);

roughness = zeros(numSamples, 1);
for si=1:numSamples
    depthMap = loadDepthMap(srcData(si,:), datasetIndex, depthList, radius);
    roughness(si) = estiRoughness(depthMap);
end

roughData = [roughness srcData];
roughSorted = sortrows(roughData, 1);
dstData = roughSorted(:,2:end);
end

%----------------------------------------------
function depthMap = loadDepthMap(datarow, datasetIndex, depthList, radius_cm)

global dataIndices
model = struct('frame', datarow(dataIndices.frame), 'pixel', datarow(dataIndices.pixel), ...
                'point', datarow(dataIndices.point), 'normal', datarow(dataIndices.normal), ...
                'praxis', datarow(dataIndices.praxis));

datasetPath = workingDir(datasetIndex);
filename = sprintf('%s/%s', datasetPath, depthList{model.frame,1});

factor = 5000; % for the 16-bit PNG files
radius_m = radius_cm/100; % cm -> m
pixel = model.pixel + [1 1]; % zero-base pixel -> one-base pixel

depthimg = double(imread(filename))/factor;
% [640 480] --> [320 240]
depthimg = scaleDepthImage(depthimg);
depthMap = extractNeighborDepths(depthimg, pixel, radius_m);
end

function dstDepth = scaleDepthImage(srcDepth)

tgt_width = 320;
tgt_height = 240;
dstDepth = zeros(tgt_height, tgt_width);
scale = size(srcDepth,2) / tgt_width;
ofs = 1;

for y=1:tgt_height
    for x=1:tgt_width
        vcnt=0;
        depth=0;
        if srcDepth(y*scale, x*scale) > 0
            depth = depth + srcDepth(y*scale, x*scale);
            vcnt = vcnt+1;
        end
        if srcDepth(y*scale-ofs, x*scale) > 0
            depth = depth + srcDepth(y*scale-ofs, x*scale);
            vcnt = vcnt+1;
        end
        if srcDepth(y*scale, x*scale-ofs) > 0
            depth = depth + srcDepth(y*scale, x*scale-ofs);
            vcnt = vcnt+1;
        end
        if srcDepth(y*scale-ofs, x*scale-ofs) > 0
            depth = depth + srcDepth(y*scale-ofs, x*scale-ofs);
            vcnt = vcnt+1;
        end
        depth = depth/vcnt;
        dstDepth(y,x) = depth;
    end
end

end

function neibDepth = extractNeighborDepths(depthimg, pixel, radius)

fu = 468.60/2;  % focal length x
fv = 468.61/2;  % focal length y
cu = 318.27/2;  % optical center x
cv = 243.99/2;  % optical center y

im_height = size(depthimg,1);
im_width = size(depthimg,2);
ctdepth = depthimg(pixel(2), pixel(1));

pxradius = round(radius*0.8 / ctdepth * fu);
bound = [max(pixel(1)-pxradius, 1), min(pixel(1)+pxradius,im_width), ...
            max(pixel(2)-pxradius, 1), min(pixel(2)+pxradius,im_height)];
neibDepth = zeros(bound(2)-bound(1)+1, bound(4)-bound(3)+1);

for u = bound(1):bound(2)
    for v = bound(3):bound(4)
        smdepth = smoothDepth(depthimg, [u v]);
        didc = [u v] - [bound(1) bound(3)] + [1 1];
        neibDepth(didc(2), didc(1)) = smdepth;
    end
end
end

function smdepth = smoothDepth(depthImg, pixel)
persistent kernel
if isempty(kernel)
    kernel = [0.05 0.15 0.05; 0.15 0.2 0.15; 0.05 0.15 0.05];
    if sum(sum(kernel))~=1
        error('kernel sum ~= 1')
    end
end

smcount=0;
depthsum = 0;
weightsum = 0;
ctdepth = depthImg(pixel(2), pixel(1));
for v=1:3
    for u=1:3
        y = pixel(2)-2+v;
        x = pixel(1)-2+u;
        if abs(depthImg(y,x)-ctdepth) < 0.01
            depthsum = depthsum + depthImg(y,x)*kernel(v,u);
            weightsum = weightsum + kernel(v,u);
            smcount = smcount+1;
        end
    end
end
smdepth = depthsum / weightsum;
end

function roughness = estiRoughness(depthMap)
mapWidth = size(depthMap,2);
mapHeight = size(depthMap,1);
roughness = 0;
count = 0;

for u=2:mapWidth-1
    for v=2:mapHeight-1
        roughness = roughness + roughnessFromThreeDepths(depthMap(u-1:u+1,v));
        roughness = roughness + roughnessFromThreeDepths(depthMap(u,v-1:v+1));
        count = count + 2;
    end
end
roughness = roughness/count;
end

function roughness = roughnessFromThreeDepths(depths)
highval = max(depths(1), depths(3)) + 0.003;
lowval = min(depths(1), depths(3)) - 0.003;
centerval = depths(2);
roughness = 0;

if centerval > highval
    roughness = centerval - highval;
elseif centerval < lowval
    roughness = lowval - centerval;
end
end
































