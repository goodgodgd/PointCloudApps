function [ptcloud, imgpxs] = loadPointCloud(depthFileName, pixel)

global radius
radius_cm = radius;
radius_m = radius_cm*0.01;
minpts = 20;

[depthmap, boundbox] = loadDepthMapScaled(depthFileName, pixel, radius_cm*2);
if nargout==2
    [points, imgpxs] = convertToPointCloud(depthmap, boundbox, pixel);
else
    points = convertToPointCloud(depthmap, boundbox, pixel);
end
% check validity
if size(points,1) < minpts
    ME = MException('shapeDistance:loadPointCloud', 'insufficient points %d', size(points,1));
    throw(ME)
end

% normals = pcnormals(ptcloud, 20);
% normals = alignNormals(points, normals);
normals = computeNormals(points, radius_m, 0.02);
ptcloud = pointCloud(points, 'Normal', normals);
indices = findNeighborIndices(points, points(1,:), radius_m);
ptcloud = select(ptcloud, indices);
if nargout==2
    imgpxs = imgpxs(indices,:);
end

if length(indices) < minpts
    ME = MException('shapeDistance:loadPointCloud', 'insufficient points %d', length(indices));
    throw(ME)
end
end

function [points, imgpxs] = convertToPointCloud(depthmap, boundbox, pixel)
global fu fv cu cv deadDepth
points = zeros(size(depthmap,1)*size(depthmap,2),3);
if nargout==2
    imgpxs = zeros(size(points,1),2);
end
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
        points(count,2) = -(u - cu) * depth / fu;
        points(count,3) = -(v - cv) * depth / fv;
        if nargout==2
            imgpxs(count,:) = [u v];
        end
    end
end

% replace the first with the center point
indices = 1:count;
indices(1) = ctindex;
indices(ctindex) = 1;
points = points(indices,:);
if nargout==2
    imgpxs = imgpxs(indices,:);
end
end

function normals = computeNormals(points, inlier_radius, normal_radius)

numpts = size(points,1);
normals = zeros(numpts,3);
centerpt = points(1,:);

for i=1:numpts
    if norm(points(i,:) - centerpt) > inlier_radius
        continue
    end
    neibindices = findNeighborIndices(points, points(i,:), normal_radius);
    if length(neibindices) < 5
        neibindices = findNeighborIndices(points, points(i,:), normal_radius*2);
        if length(neibindices) < 5
            continue;
        end
    end
    
    neighbors = points(neibindices,:);
    center = mean(neighbors);
    diff = neighbors - repmat(center, size(neighbors,1), 1);
    cova = diff'*diff;
    [evec, eval] = eig(cova);
    [~, index] = min( abs(real(diag(eval))) );
    normals(i,:) = evec(:,index)';
    normals(i,:) = normals(i,:) / norm(normals(i,:));
    
    if dot(points(i,:), normals(i,:)) > 0
        normals(i,:) = -normals(i,:);
    end
end
end

function neighbors = findNeighborIndices(points, query, radius)
diffs = points - repmat(query,size(points,1),1);
norms = sum(diffs.*diffs, 2);
neighbors = find(norms < radius*radius);
end

