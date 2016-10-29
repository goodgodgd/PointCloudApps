global dataIndices
global distThreshForGradWeight

frame = 1;
pixel = 2:3;
point = 4:6;
normal = 7:9;
praxis = 10:12;

prcvSize=2; pcwgSize=4; spinSize=153; fpfhSize=33; shotSize=352; trisSize=spinSize*3;
prcv = praxis(end)+1:praxis(end)+prcvSize;
pcwg = praxis(end)+1:praxis(end)+pcwgSize;
fpfh = pcwg(end)+1:pcwg(end)+fpfhSize;
shot = fpfh(end)+1:fpfh(end)+shotSize;
spin = shot(end)+1:shot(end)+spinSize;
tris = shot(end)+1:shot(end)+trisSize;
totdescr = prcv(1):tris(end);
descrs = containers.Map({1, 2, 3, 4, 5, 6}, {prcv, pcwg, fpfh, shot, spin, tris});

dataIndices = struct('frame', frame, 'pixel', pixel, 'point', point, 'normal', normal, 'praxis', praxis,...
                     'descrs', descrs, 'total', totdescr);

distThreshForGradWeight = containers.Map({4, 5, 6}, {0.002, 0.003, 0.003});
