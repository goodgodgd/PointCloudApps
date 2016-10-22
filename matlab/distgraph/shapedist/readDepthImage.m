function pointCloud = readDepthImage(filename, pixel, radius)

fu = 468.60/2;  % focal length x
fv = 468.61/2;  % focal length y
cu = 318.27/2;  % optical center x
cv = 243.99/2;  % optical center y
factor = 5000; % for the 16-bit PNG files
radius_m = radius/100;
pixel = pixel + [1 1]; % zero-base pixel --> one-base pixel
filename
depthImg = double(imread(filename))/factor;
% [640 480] --> [320 240]
depthImg = scaleDepthImage(depthImg);
im_height = size(depthImg,1);
im_width = size(depthImg,2);
ctdepth = depthImg(pixel(2), pixel(1));

if ctdepth < 0.05
    pointCloud = zeros(0,3);
    sprintf('%d %d pixel invalid: %f', pixel(1), pixel(2), depthImg(pixel(2), pixel(1)))
    return
end

pxradius = round(radius_m / ctdepth * fu);
pointCloud = zeros(3,1000);

count=0;
ctindex=0;
kernel = [0.05 0.15 0.05; 0.15 0.2 0.15; 0.05 0.15 0.05];
bound = [max(pixel(1)-pxradius, 2), min(pixel(1)+pxradius,im_width-1), ...
            max(pixel(2)-pxradius, 2), min(pixel(2)+pxradius,im_height-1)];
for u = bound(1):bound(2)
    for v = bound(3):bound(4)
        count=count+1;
        smdepth = smoothDepth(depthImg, [u v], kernel);
        if u==pixel(1) && v==pixel(2)
            ctindex = count;
%             depthCompare = [depthImg(v,u), smdepth]
        end
        pointCloud(1,count) = smdepth;
        pointCloud(2,count) = -(u - cu - 1) * smdepth / fu;
        pointCloud(3,count) = -(v - cv - 1) * smdepth / fv;
    end
end

% replace the first with the center point
indices = 1:count;
indices(1) = ctindex;
indices(ctindex) = 1;
pointCloud = pointCloud(:,indices);

% filter points out of radius
dist = zeros(1,size(pointCloud,2));
for i=1:count
    dist(i) = norm(pointCloud(:,1) - pointCloud(:,i), 2);
end
pointCloud = pointCloud(:,dist<radius_m);

end

%----------------------------------------------
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

function smdepth = smoothDepth(depthImg, pixel, kernel)

depthsum = 0;
weightsum = 0;
ctdepth = depthImg(pixel(2), pixel(1));
for v=1:3
    for u=1:3
        y = v-2+pixel(2);
        x = u-2+pixel(1);
        if abs(depthImg(y,x)-ctdepth) < 0.01
            depthsum = depthsum + depthImg(y,x)*kernel(v,u);
            weightsum = weightsum + kernel(v,u);
        end
    end
end

smdepth = depthsum / weightsum;
end