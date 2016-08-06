clc
clear

% A = randn(5,100)+3;
% B = randn(5,100)*3;
% sampleKL(A, B)
% 
% A = randn(20,100)+3;
% B = randn(20,100)*3;
% sampleKL(A, B)
% 
A = randn(100,10)+3;
B = randn(100,10)*3;
d = bhattacharyya(A,B)

addpath('../utils')
gd = gbhattacharyya(A,B)

