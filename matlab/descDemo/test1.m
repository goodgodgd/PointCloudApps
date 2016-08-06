clc
clear

'Descriptor Test'

% true shape parameter
shprm = [0.3 0.1];

% generate 3D points of z = 2x^2 + y^2
itv = 0.02;
x = reshape(((-10:10)'*itv)*ones(1,21), 21*21, 1);
y = reshape((((-10:10)'*itv)*ones(1,21))', 21*21, 1);
z = shprm(1)*x.*x + shprm(2)*y.*y;
Porigin = [x'; y'; z'];
n = size(x);
% rotate 
rotang = [0.5 0 0];
R = angle2dcm(rotang(1),rotang(2),rotang(3))
P = R*Porigin;

normal = [0; 0; 1];
normal = R*normal

% prepare constant matrices
d = 6;
F = zeros(n,d);
% F(:,1) = x.*x;
% F(:,2) = y.*y;
% F(:,3) = z.*z;
% F(:,4) = 2*x.*y;
% F(:,5) = 2*y.*z;
% F(:,6) = 2*z.*x;
F(:,1) = P(1,:)'.*P(1,:)';
F(:,2) = P(2,:)'.*P(2,:)';
F(:,3) = P(3,:)'.*P(3,:)';
F(:,4) = 2*P(1,:)'.*P(2,:)';
F(:,5) = 2*P(2,:)'.*P(3,:)';
F(:,6) = 2*P(3,:)'.*P(1,:)';

G = zeros(3,d);
G(1,[1 4 6]) = normal;
G(2,[4 2 5]) = normal;
G(3,[6 5 3]) = normal;

Xn = P'*normal;
zv = zeros(3,1);

% solve without CVX
M = [F'*F G'; G zeros(3,3)];
q = [F'*Xn; zeros(3,1)];
Ab = [M, q]
%[Abvec, Abval] = eig(M)

yl = M\q;
ty = yl(1:6)

'Result'
(F*ty-Xn)'*(F*ty-Xn)
(G*ty)'

Aopt = zeros(3,3);
Aopt(1,1) = ty(1);
Aopt(2,2) = ty(2);
Aopt(3,3) = ty(3);
Aopt(1,2) = ty(4);
Aopt(2,1) = ty(4);
Aopt(2,3) = ty(5);
Aopt(3,2) = ty(5);
Aopt(3,1) = ty(6);
Aopt(1,3) = ty(6);
Aopt

[evec eval] = eig(Aopt);
eval = diag(eval)';
[abeval sidc] = sort(abs(eval), 2, 'descend');
Aevec = evec(:,sidc);
Aeval = eval(sidc);
Aevec
Aeval

return;

Abcpp = [      0.17024      0.07321     0.022308  -3.4925e-09     0.079276  -4.6566e-10            0            0            0     0.060558;
      0.07321      0.10143     0.030109  -2.5611e-09      0.10974   3.4925e-10            0     -0.47943            0     0.035084;
     0.022308     0.030109    0.0094497            0     0.033507   5.8208e-11            0            0      0.87758     0.010638;
  -3.4925e-09  -2.5611e-09            0      0.29284  -1.1642e-09      0.15855     -0.47943            0            0  -4.1395e-08;
     0.079276      0.10974     0.033507  -1.1642e-09      0.12044            0            0      0.87758     -0.47943     0.038072;
  -4.6566e-10   3.4925e-10   5.8208e-11      0.15855            0      0.08923      0.87758            0            0   3.4244e-08;
            0            0            0     -0.47943            0      0.87758            0            0            0            0;
            0     -0.47943            0            0      0.87758            0            0            0            0            0;
            0            0      0.87758            0     -0.47943            0            0            0            0            0];
Abdiff = Ab - Abcpp;
sumdf = sum(sum(abs(Abdiff)))
ylcpp = Abcpp(:,1:9) \ Abcpp(:,10);
tycpp = ylcpp(1:6);
tydf = ty - tycpp;
tydf'

Aopt = zeros(3,3);
Aopt(1,1) = ty(1);
Aopt(2,2) = ty(2);
Aopt(3,3) = ty(3);
Aopt(1,2) = ty(4);
Aopt(2,1) = ty(4);
Aopt(2,3) = ty(5);
Aopt(3,2) = ty(5);
Aopt(3,1) = ty(6);
Aopt(1,3) = ty(6);

[evec eval] = eig(Aopt)
