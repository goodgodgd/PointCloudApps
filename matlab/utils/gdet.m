function dtm = gdet(mat,prc)
if nargin==3
    precision = prc;
else
    precision = 0.001;
end

eval = eig(mat);
dtm = 1;
for i=1:size(eval,1)
    if abs(eval(i)) > precision
        dtm = dtm * eval(i);
    end
end

dtm = real(dtm);
end
