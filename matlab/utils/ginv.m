function imat = ginv(mat,prc)
if nargin==3
    precision = prc;
else
    precision = 0.001;
end

[evec eval] = eig(mat);
invEval = zeros(size(eval));
for i=1:size(eval,1)
    if abs(eval(i,i)) > precision
        invEval(i,i) = 1/eval(i,i);
    end
end
imat = evec*invEval*evec';
