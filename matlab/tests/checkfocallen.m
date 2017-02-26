fu = 468.60/2;  % focal length x
fv = 468.61/2;  % focal length y
cu = 318.27/2 + 1;  % optical center x + one base coordinate
cv = 243.99/2 + 1;  % optical center y + one base coordinate

pixel = [32 74]
point1 = [1.7520 0.3963 0.1444]
point2 = [1.7549    0.9597    0.3670]

fu1 = point1(1)/point1(2)*(pixel(1) - cu)
fv1 = point1(1)/point1(3)*(pixel(2) - cv)

fu2 = point2(1)/point2(2)*(pixel(1) - cu)
fv2 = point2(1)/point2(3)*(pixel(2) - cv)
