function optimizeGradientWeight()

% compute difference between samples and distance between raw descriptors
sampleDiffAbs = abs(samples(indexPairs(:,1),:) - samples(indexPairs(:,2),:));
for dtype=1:6
    descrDists(:,dtype) = sum(sampleDiffAbs(:, dataIndices.descrs(dtype)), 2);
end

% compute optimal gradient weight
pcwg = 2;
validIndices = (descrDists < distThreshForGradWeight(radius));
gradWeight = gradientWeight(sampleDiffAbs(validIndices, dataIndices.descrs(pcwg)), ...
                            descrDists(validIndices))
gradWeight = 0.3

end


function weight = gradientWeight(descDiffAbs, shapeDists)
SDmat = repmat(shapeDists,1,4);
wdiff = descDiffAbs./SDmat;
len = size(wdiff,1);
warning('off');
cvx_begin quiet
    variables w(1,1) a(1,1);
%     minimize( norm(wdiff*[1 1 w w]' - a*ones(len,1), 1) );
    minimize( norm(wdiff*[a a w w]' - ones(len,1), 1) );
    subject to
        w >= 0;
        a >= 0;
cvx_end
warning('on');
grad = [w a mean(wdiff*[a a w w]' - ones(len,1))]
weight = w/a;
end


