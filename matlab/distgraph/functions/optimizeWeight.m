function [weight ratio] = optimizeWeight(sampleDiffs, shapeDists)
% clc
% clear
% sampleDiffs = rand(1000);
% shapeDists = rand(1000,1)*2;

initGlobals;
global dataIndices

if size(sampleDiffs,1)~=size(shapeDists,1)
    error('inconsistent data size')
end

weight = zeros(1,size(sampleDiffs,2));
ratio = zeros(1,6);
numData = length(shapeDists);


for dtype=1:6
    indices = dataIndices.descrs(dtype);
    SDmat = repmat(shapeDists,1,length(indices));
    wdiff = sampleDiffs(:,indices)./SDmat;

    dim = size(wdiff,2);
    len = size(wdiff,1);
    cvx_begin quiet
        variables w(dim,1) a(1,1);
        minimize( norm(wdiff*w - a*ones(len,1)) );
        subject to
            w >= 0;
            ones(1,dim)*w==1;
            a >= 0;
    cvx_end
    [a norm(wdiff*w - a*ones(len,1)) norm(wdiff*w - a*ones(len,1))/a]
end
