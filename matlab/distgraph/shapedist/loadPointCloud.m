function pointCloud = loadPointCloud(filename)

fu = 525.0;  % focal length x
fv = 525.0;  % focal length y
cu = 319.5;  % optical center x
cv = 239.5;  % optical center y
factor = 5000; % for the 16-bit PNG files
depth = double(imread(filename));
im_height = size(depth,1);
im_width = size(depth,2);
pointCloud = zeros(3, im_height*im_width);

for v = 1:im_height
    for u = 1:im_width
        pxdepth = depth(v,u) / factor;
        pointCloud(1,v*im_width+u) = pxdepth;
        pointCloud(2,v*im_width+u) = -(u - cu) * pxdepth / fu;
        pointCloud(3,v*im_width+u) = -(v - cv) * pxdepth / fv;
    end
end

pointCloud = pointCloud(:, pointCloud(1,:)>0.05);
size(pointCloud)
figure(1)
plot3(pointCloud(1,:), pointCloud(2,:), pointCloud(3,:), 'b.')
xlabel('x')
ylabel('y')
zlabel('z')