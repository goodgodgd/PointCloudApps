function dkl = sampleKL(A,B)

Ai.mu = mean(A,2);
Ai.sigma = covar(A);
Bi.mu = mean(B,2);
Bi.sigma = covar(B);
dkl = KL(Ai,Bi);
