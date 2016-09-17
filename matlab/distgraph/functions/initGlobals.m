global dataIndices

frame = 1;
pixel = 2:3;
normal = 4:6;
praxis = 7:9;

prcvSize=2; pcwgSize=4; spinSize=153; fpfhSize=33; shotSize=352; trisSize=spinSize*3;
prcv = praxis(end)+1:praxis(end)+prcvSize;
pcwg = praxis(end)+1:praxis(end)+pcwgSize;
fpfh = pcwg(end)+1:pcwg(end)+fpfhSize;
shot = fpfh(end)+1:fpfh(end)+shotSize;
spin = shot(end)+1:shot(end)+spinSize;
tris = shot(end)+1:shot(end)+trisSize;
totdescr = prcv(1):tris(end);
descrs = containers.Map({1, 2, 3, 4, 5, 6}, {prcv, pcwg, spin, fpfh, shot, tris});

dataIndices = struct('frame', frame, 'pixel', pixel, 'normal', normal, 'praxis', praxis,...
                     'descrs', descrs, 'total', totdescr);
