function curvature()
clc
clear

quadCurve(0,0)
pause
quadCurve(1,0)
pause
ellipticParaboloid(2,1)
pause
quadCurve(-2,1)
pause

end

function ellipticParaboloid(a, b)
fx = @(u,v) a*u.*cos(v);
fy = @(u,v) b*u.*sin(v);
fz = @(u,v) 0.5*u.^2;

u = linspace(0,5, 30);
v = linspace(-pi,pi, 30);
[uu,vv] = meshgrid(u, v);

figure(1)
surf(fx(uu,vv), fy(uu,vv),  fz(uu,vv));
axis equal;
end

function quadCurve(a, b)
if a~=0 && b==0
    [X,Y] = meshgrid(-5:.25:5, -5:.5:5);
elseif a<0
    a
    [X,Y] = meshgrid(-5:.5:5, -7:.5:7);
else
    [X,Y] = meshgrid(-5:.5:5);
end
Z = (a*X.*X + b*Y.*Y)*0.5;

figure(1)
surf(X,Y,Z)
axis equal
end

