function [depthmap, boundbox] = loadDepthMapScaled(filename, pixel, radius_cm)

global deadDepth depthFactor
radius_m = radius_cm/100; % cm -> m

% read image
depthimg = double(imread(filename))/depthFactor;
% scale image: [640 480] -> [320 240]
depthimg = scaleDepthImage(depthimg);
if depthimg(pixel(2), pixel(1)) < deadDepth
    ME = MException('shapeDistance:loadDepthMapScaled', ...
                    '%d %d pixel invalid: %f', pixel(1), pixel(2), depthimg(pixel(2), pixel(1)));
    throw(ME)
end

boundbox = findBoundingBox(depthimg, pixel, radius_m);
depthmap = getSmoothedDepths(depthimg, boundbox);
end

function dstDepth = scaleDepthImage(srcDepth)
global imwidth imheight
dstDepth = zeros(imheight, imwidth);
scale = size(srcDepth,2) / imwidth;

for y=0:imheight-2
    for x=0:imwidth-2
        vcnt=0;
        depth=0;
        ysc = y*scale + 1;
        xsc = x*scale + 1;
        if srcDepth(ysc, xsc) > 0
            depth = depth + srcDepth(ysc, xsc);
            vcnt = vcnt+1;
        end
        if srcDepth(ysc, xsc+1) > 0
            depth = depth + srcDepth(ysc, xsc+1);
            vcnt = vcnt+1;
        end
        if srcDepth(ysc+1, xsc) > 0
            depth = depth + srcDepth(ysc+1, xsc);
            vcnt = vcnt+1;
        end
        if srcDepth(ysc+1, xsc+1) > 0
            depth = depth + srcDepth(ysc+1, xsc+1);
            vcnt = vcnt+1;
        end
        if vcnt>0
            dstDepth(y+1,x+1) = depth/vcnt;
        end
    end
end
end

function boundbox = findBoundingBox(depthimg, pixel, radius_m)
global fu
im_height = size(depthimg,1);
im_width = size(depthimg,2);
ctdepth = depthimg(pixel(2), pixel(1));

pxradius = round(radius_m / ctdepth * fu);
boundbox = [max(pixel(1)-pxradius, 2), min(pixel(1)+pxradius,im_width-1), ...
            max(pixel(2)-pxradius, 2), min(pixel(2)+pxradius,im_height-1)];
end

function depthmap = getSmoothedDepths(depthimg, boundbox)
depthmap = zeros(boundbox(2)-boundbox(1)+1, boundbox(4)-boundbox(3)+1);
for u = boundbox(1):boundbox(2)
    for v = boundbox(3):boundbox(4)
        didc = [u v] - [boundbox(1) boundbox(3)] + [1 1];
        smdepth = smoothDepth(depthimg, [u v]);
        depthmap(didc(2), didc(1)) = smdepth;
    end
end
end

function smdepth = smoothDepth(depthimg, pixel)
persistent kernel
global deadDepth
if isempty(kernel)
    kernel = [0.05 0.15 0.05; 0.15 0.2 0.15; 0.05 0.15 0.05];
%     kernel = [0 0 0; 0 1 0; 0 0 0];
    if sum(sum(kernel))~=1
        error('kernel sum ~= 1')
    end
end

smcount=0;
depthsum = 0;
weightsum = 0;
ctdepth = depthimg(pixel(2), pixel(1));
for v=1:3
    for u=1:3
        y = pixel(2)+v-2;
        x = pixel(1)+u-2;
        if depthimg(y,x) > deadDepth && abs(depthimg(y,x)-ctdepth) < 0.01
            depthsum = depthsum + depthimg(y,x)*kernel(v,u);
            weightsum = weightsum + kernel(v,u);
            smcount = smcount+1;
        end
    end
end

if weightsum < 0.01
    smdepth = 0;
else
    smdepth = depthsum / weightsum;
end
end

