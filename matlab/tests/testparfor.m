clear
clc

c = zeros(1000,3);
tic
parfor i = 1:3
    c(:,i) = eig(rand(1000));
    sprintf('print %d', i)
    t = datetime('now');
    [h,m,s] = hms(t);
    [h m s]
end
toc