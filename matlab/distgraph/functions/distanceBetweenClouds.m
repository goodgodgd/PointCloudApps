function distance = distanceBetweenClouds(pcmodel, pcquery)
distance = pointToPlaneDist(pcmodel, pcquery);

% global normalDistWeight
% distance_p2q = pointToPlaneDist(pcmodel, pcquery);
% distance_q2p = pointToPlaneDist(pcquery, pcmodel);
% 
% if distance_p2q(1)+distance_p2q(2)*normalDistWeight < distance_q2p(1)+distance_q2p(2)*normalDistWeight
%     distance = distance_q2p;
% else
%     distance = distance_p2q;
% end
end

function distance = pointToPlaneDist(pcrefer, pccompa)
pdistsum = 0;
pdcnt = 0;
ndistsum = 0;
ndcnt = 0;

for cidx=1:pccompa.Count
    [ridx, ~] = findNearestNeighbors(pcrefer, pccompa.Location(cidx,:), 1);
    if abs(1 - norm(pccompa.Normal(cidx,:))) > 0.1
        continue;
    end
    pdistsum = pdistsum + abs(dot(pccompa.Normal(cidx,:), ...
                            pccompa.Location(cidx,:) - pcrefer.Location(ridx,:)));
    pdcnt = pdcnt+1;
    
    if abs(1 - norm(pcrefer.Normal(ridx,:))) > 0.1
        continue;
    end
    ndistsum = ndistsum + acos(dot(pccompa.Normal(cidx,:), pcrefer.Normal(ridx,:)));
    ndcnt = ndcnt+1;
end

if pdcnt < 20 || ndcnt < 20
    ME = MException('shapeDistance:pointToPlaneDist', 'point pairs %d, normal pairs %d', ...
                    pdcnt, ndcnt);
    throw(ME)
end

% distcnt = [pdistsum ndistsum pdcnt ndcnt]
distance = [pdistsum/pdcnt ndistsum/ndcnt];
end
