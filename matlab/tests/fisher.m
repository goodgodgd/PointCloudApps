function result = fisher(A, B)

CA = covar(A);
[evec, eval] = eig(CA);
aval = diag(real(eval));

CB = covar(B);
[evec, eval] = eig(CB);
bval = diag(real(eval));

mA = mean(A,2);
mB = mean(B,2);
SW = CA + CB;
SB = covar([mA mB]);

[evec, eval] = eig(SB/SW);
eval = diag(real(eval));
result = [eval aval bval];