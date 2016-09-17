function pointCloud = readDepthImage(filename, pixel, radius)

fu = 525.0;  % focal length x
fv = 525.0;  % focal length y
cu = 319.5;  % optical center x
cv = 239.5;  % optical center y
factor = 5000; % for the 16-bit PNG files
radius_m = radius/100;
filename
depth = double(imread(filename));
% depth(100:110, 101:104)
ctdepth = depth(pixel(2), pixel(1))/factor;

if ctdepth < 0.05
    pointCloud = zeros(0,3);
    sprintf('%d %d pixel invalid', pixel(1), pixel(2))
    return
end

pxradius = round(radius_m / ctdepth * fu)
pointCloud = zeros(3,1000);

count=0;
ctindex=0;
for u = pixel(1)-pxradius:pixel(1)+pxradius
    for v = pixel(2)-pxradius:pixel(2)+pxradius
        count=count+1;
        if u==pixel(1) && v==pixel(2)
            ctindex = count;
        end
        pointCloud(1,count) = depth(v,u) / factor;
        pointCloud(2,count) = -(u - cu) * pointCloud(1,count) / fu;
        pointCloud(3,count) = -(v - cv) * pointCloud(1,count) / fv;
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
