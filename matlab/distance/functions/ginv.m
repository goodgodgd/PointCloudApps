function imat = ginv(mat, numeigs)

validEigs = size(mat,1);
if nargin==2
    validEigs = numeigs;
end

[evec eval] = eig(mat);
evalMag = real(diag(eval));
[sortEval, sortIdc] = sort(evalMag, 'descend');

invEval = zeros(size(eval));
for i=1:validEigs
    ei = sortIdc(i);
    invEval(ei,ei) = 1/eval(ei,ei);
end
imat = evec*invEval*evec';

end
