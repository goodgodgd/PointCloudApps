clc
clear

% true shape parameter
shprm = [0.3 0.1];

% generate 3D points of z = 2x^2 + y^2
% n = 500;
% x = (rand(n,1) - 0.5)*4;
% y = (rand(n,1) - 0.5)*4;
x = reshape(((-10:10)'*0.2)*ones(1,21), 21*21, 1);
y = reshape((((-10:10)'*0.2)*ones(1,21))', 21*21, 1);
z = shprm(1)*x.*x + shprm(2)*y.*y;
Porigin = [x'; y'; z'];
n = size(x);

%{
% show original points
figure(1)
plot3(Porigin(1,:), Porigin(2,:), Porigin(3,:), '.')
%}

% rotate 
rotang = [0.3 0.7 0.5];
%R = angle2dcm(rotang(1),rotang(2),rotang(3));
R = eye(3)
P = R*Porigin;

%{
% show rotated points
figure(2)
hold off
plot3(P(1,:), P(2,:), P(3,:), 'c.')
hold on
%}

% compute expected solution
Aexp = zeros(3,3);
Aexp(1,1) = shprm(1);
Aexp(2,2) = shprm(2);
Aexp = R*Aexp*R';
[Eevec Eeval] = eig(Aexp);
Eeval = diag(Eeval)';
Aexp

% compute mean vector
mu = mean(P,2);
% compute covariance matrix
sig = (P-mu*ones(1,n))*(P-mu*ones(1,n))';
% compute normal vector from covariance
[evec eval] = eig(sig);
eval = diag(eval)';
[abeval sidc] = sort(abs(eval), 2, 'descend');
Sevec = evec(:,sidc);
Seval = eval(sidc);
% get normal vector
% normal = Sevec(:,3);
normal = R(:,3);
normal

%{
% draw rotation axis
plot3([0 R(1,1)]*1, [0 R(2,1)]*1, [0 R(3,1)]*1, 'r', 'LineWidth', 2)
plot3([0 R(1,2)]*1, [0 R(2,2)]*1, [0 R(3,2)]*1, 'g', 'LineWidth', 2)
plot3([0 R(1,3)]*1, [0 R(2,3)]*1, [0 R(3,3)]*1, 'b', 'LineWidth', 2)
% draw eigenvectors
plot3([0 Sevec(1,1)]*2, [0 Sevec(2,1)]*2, [0 Sevec(3,1)]*2, 'r:')
plot3([0 Sevec(1,2)]*2, [0 Sevec(2,2)]*2, [0 Sevec(3,2)]*2, 'g:')
plot3([0 Sevec(1,3)]*2, [0 Sevec(2,3)]*2, [0 Sevec(3,3)]*2, 'b:')
%}

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
G(1,1) = normal(1);
G(1,4) = normal(2);
G(1,6) = normal(3);

G(2,2) = normal(2);
G(2,4) = normal(1);
G(2,5) = normal(3);

G(3,3) = normal(3);
G(3,5) = normal(2);
G(3,6) = normal(1);

Xn = P'*normal;
zv = zeros(3,1);

% solve without CVX
M = [F'*F G'; G zeros(3,3)];
q = [F'*Xn; zeros(3,1)];
Ab = [M, q]

yl = M\q;
ty = yl(1:6);

'Result'
(F*ty-Xn)'*(F*ty-Xn)
G*ty

ey = zeros(6,1);
ey(1) = Aexp(1,1);
ey(2) = Aexp(2,2);
ey(3) = Aexp(3,3);
ey(4) = Aexp(1,2);
ey(5) = Aexp(2,3);
ey(6) = Aexp(1,3);

(F*ey-Xn)'*(F*ey-Xn)
G*ey


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

[evec eval] = eig(Aopt);
eval = diag(eval)';
[abeval sidc] = sort(abs(eval), 2, 'descend');
Aevec = evec(:,sidc);
Aeval = eval(sidc);

% normal'
% Aevec(:,3)'

R
Eevec
Eeval
Sevec
Seval
Aevec
Aeval

%{
plot3([0 Aevec(1,1)]*2, [0 Aevec(2,1)]*2, [0 Aevec(3,1)]*2, 'r')
plot3([0 Aevec(1,2)]*2, [0 Aevec(2,2)]*2, [0 Aevec(3,2)]*2, 'g')
plot3([0 Aevec(1,3)]*2, [0 Aevec(2,3)]*2, [0 Aevec(3,3)]*2, 'b')
%}

