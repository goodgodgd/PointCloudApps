function result = covar(X)

mX = mean(X,2);
dX = X - mX*ones(1,size(X,2));
result = dX*dX';
result = result/size(X,2);
