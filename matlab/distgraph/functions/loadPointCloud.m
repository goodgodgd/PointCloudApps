function ptcloud = loadPointCloud(depthFileName, pixel, radius_cm, centerpt)

[depthmap, boundbox] = loadDepthMapScaled(depthFileName, pixel, radius_cm*2);
points = convertToPointCloud(depthmap, boundbox, pixel);
% check validity
minpts = 20;
if size(points,1) < minpts
    ME = MException('shapeDistance:loadPointCloud', 'insufficient points %d', size(points,1));
    throw(ME)
end
% if norm(points(1,:) - centerpt) > 0.02
%     'inconsistent center coordinate'
%     checkCenter = [points(1,:), centerpt, points(1,:) - centerpt]
%     ME = MException('shapeDistance:loadPointCloud', 'wrong point cloud loaded');
%     throw(ME)
% end

% create point cloud object with normal
ptcloud = pointCloud(points);
normals = pcnormals(ptcloud, 20);
normals = alignNormals(points, normals);
ptcloud = pointCloud(points, 'Normal', normals);
radius_m = radius_cm*0.01;

% filter points within radius
[indices, dists] = findNeighborsInRadius(ptcloud, points(1,:), radius_m);
if length(indices) < minpts
    ME = MException('shapeDistance:loadPointCloud', 'insufficient points %d', length(indices));
    throw(ME)
end
ptcloud = select(ptcloud, indices);
end

function points = convertToPointCloud(depthmap, boundbox, pixel)
global fu fv cu cv deadDepth
points = zeros(size(depthmap,1)*size(depthmap,2),3);
count=0;
ctindex=0;
for u = boundbox(1):boundbox(2)
    for v = boundbox(3):boundbox(4)
        count=count+1;
        if u==pixel(1) && v==pixel(2)
            ctindex = count;
        end
        depth = depthmap(v-boundbox(3)+1, u-boundbox(1)+1);
        if depth < deadDepth
            continue
        end
        points(count,1) = depth;
        points(count,2) = -(u - cu - 1) * depth / fu;
        points(count,3) = -(v - cv - 1) * depth / fv;
    end
end

% replace the first with the center point
indices = 1:count;
indices(1) = ctindex;
indices(ctindex) = 1;
points = points(indices,:);
end

function normals = alignNormals(points, normals)
for i=1:size(points,1)
    if dot(points(i,:), normals(i,:)) < 0
        normals(i,:) = -normals(i,:);
    end
end
end
