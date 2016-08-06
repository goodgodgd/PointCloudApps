function d = gBD(X1, X2, prc)
% generalized bhattacharyya distance

mu1=mean(X1)';
C1=cov(X1);
mu2=mean(X2)';
C2=cov(X2);
C=(C1+C2)/2;

dmu = mu1 - mu2;
if nargin==3
    d = 0.125*dmu'*ginv(C,prc)*dmu + 0.5*log(gdet(C,prc)/sqrt(gdet(C1,prc)*gdet(C2,prc)));
else
    d = 0.125*dmu'*ginv(C)*dmu + 0.5*log(gdet(C)/sqrt(gdet(C1)*gdet(C2)));
end

% first = 0.125*dmu'*ginv(C)*dmu
% gc = gdet(C)
% gc1 = gdet(C1)
% gc2 = gdet(C2)
% insd = gdet(C)/sqrt(gdet(C1)*gdet(C2))
% lterm = 0.5*log(gdet(C)/sqrt(gdet(C1)*gdet(C2)))
end
