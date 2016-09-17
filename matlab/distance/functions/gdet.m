function dtm = gdet(mat, numeigs)

validEigs = size(mat,1);
if nargin==2
    validEigs = numeigs;
end

eval = eig(mat);
evalMag = real(eval);
[sortEval, sortIdc] = sort(evalMag, 'descend');

dtm = 1;
for i=1:validEigs
    ei = sortIdc(i);
    dtm = dtm*eval(ei);
end

end
