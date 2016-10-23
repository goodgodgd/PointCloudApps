clc
clear
sampleDiffs = rand(100,4);
shapeDists = rand(100,1)*2;

weight = zeros(1,size(sampleDiffs,2));
ratio = zeros(1,6);
numData = length(shapeDists);


SDmat = repmat(shapeDists,1,4);
wdiff = sampleDiffs./SDmat;

dim = size(wdiff,2);
len = size(wdiff,1);
cvx_begin quiet
    variables w(1,1) a(1,1);
    minimize( norm(wdiff*[1 1 w w]' - a*ones(len,1), 1) );
    subject to
        w >= 0;
        a >= 0;
cvx_end

summean = sum(abs(wdiff*[1 1 0 0]' - a*ones(len,1)))

result = [w a  norm(wdiff*[1 1 w w]' - a*ones(len,1), 1)]

for w=0:0.05:0.3
    [w  norm(wdiff*[1 1 w w]' - a*ones(len,1), 1)]
end