function pointCloud = readDepthImage(filename, pixel, radius)

fu = 525.0/2;  % focal length x
fv = 525.0/2;  % focal length y
cu = 319.5/2;  % optical center x
cv = 239.5/2;  % optical center y
factor = 5000; % for the 16-bit PNG files
radius_m = radius/100;
pixel = pixel + [1 1]; % zero-base pixel --> one-base pixel
filename
depthImg = double(imread(filename))/factor;
% [640 480] --> [320 240]
depthImg = scaleImage(depthImg);
im_height = size(depthImg,1);
im_width = size(depthImg,2);
ctdepth = depthImg(pixel(2), pixel(1));

if ctdepth < 0.05
    pointCloud = zeros(0,3);
    sprintf('%d %d pixel invalid: %f', pixel(1), pixel(2), depthImg(pixel(2), pixel(1)))
    return
end

pxradius = round(radius_m / ctdepth * fu)
pointCloud = zeros(3,1000);

count=0;
ctindex=0;
bound = [max(pixel(1)-pxradius, 1), min(pixel(1)+pxradius,im_width), ...
            max(pixel(2)-pxradius, 1), min(pixel(2)+pxradius,im_height)];
for u = bound(1):bound(2)
    for v = bound(3):bound(4)
        count=count+1;
        if u==pixel(1) && v==pixel(2)
            ctindex = count;
        end
        pointCloud(1,count) = depthImg(v,u);
        pointCloud(2,count) = -(u - cu) * depthImg(v,u) / fu;
        pointCloud(3,count) = -(v - cv) * depthImg(v,u) / fv;
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
size(pointCloud)

end

%----------------------------------------------
function dstDepth = scaleImage(srcDepth)

tgt_width = 320;
tgt_height = 240;
dstDepth = zeros(tgt_height, tgt_width);
scale = size(srcDepth,2) / tgt_width;
ofs = round(scale/2);

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