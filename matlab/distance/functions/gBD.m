function d = gBD(X1, X2)
% generalized bhattacharyya distance

prc = 0.0001;

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
% d = 0.125*dmu'*ginv(C,numValid)*dmu + ...
%         0.5*log(gdet(C,numValid)/sqrt(gdet(C1,numValid)*gdet(C2,numValid)));
d = 0.125*dmu'*ginv(C,numValid)*dmu + ...
    0.5*(glogdet(C,numValid) - 0.5*(glogdet(C1,numValid) + glogdet(C2,numValid)));
if isinf(d)
    error('infinite distance')
end

end
