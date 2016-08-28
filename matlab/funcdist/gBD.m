function d = gBD(X1, X2)
% generalized bhattacharyya distance

prc = 0.001;

mu1=mean(X1)';
C1=cov(X1);
mu2=mean(X2)';
C2=cov(X2);
C=(C1+C2)/2;

eval = eig(C1);
numValid = sum(eval>prc);
eval = eig(C2);
numValid = min(numValid, sum(eval>prc));
eval = eig(C);
numValid = min(numValid, sum(eval>prc));

dmu = mu1 - mu2;
d = 0.125*dmu'*ginv(C,numValid)*dmu + 0.5*log(gdet(C,numValid)/sqrt(gdet(C1,numValid)*gdet(C2,numValid)));
if isinf(d)
    deterr1 = [size(C,1), numValid, dmu'*ginv(C,numValid)*dmu]
    deterr2 = [gdet(C,numValid), gdet(C1,numValid), gdet(C2,numValid)]
end

% first = 0.125*dmu'*ginv(C)*dmu
% gc = gdet(C)
% gc1 = gdet(C1)
% gc2 = gdet(C2)
% insd = gdet(C)/sqrt(gdet(C1)*gdet(C2))
% lterm = 0.5*log(gdet(C)/sqrt(gdet(C1)*gdet(C2)))
end
