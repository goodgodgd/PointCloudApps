function logdtm = glogdet(mat, numeigs)

validEigs = size(mat,1);
if nargin==2
    validEigs = numeigs;
end

eval = eig(mat);
evalMag = real(eval);
[sortEval, sortIdc] = sort(evalMag, 'descend');

logdtm = 0;
for i=1:validEigs
    ei = sortIdc(i);
    logdtm = logdtm + log(eval(ei));
end

end
