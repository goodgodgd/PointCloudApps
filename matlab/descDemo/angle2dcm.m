
function R = angle2dcm(rx, ry, rz)

R = eye(3);
R2d = [cos(rx), -sin(rx); sin(rx), cos(rx)];
Rtmp = eye(3);
Rtmp(2:3,2:3) = R2d;
R = Rtmp*R;

R2d = [cos(ry), -sin(ry); sin(ry), cos(ry)];
Rtmp = eye(3);
Rtmp([3 1],[3 1]) = R2d;
R = Rtmp*R;

R2d = [cos(rz), -sin(rz); sin(rz), cos(rz)];
Rtmp = eye(3);
Rtmp(1:2,1:2) = R2d;
R = Rtmp*R;
