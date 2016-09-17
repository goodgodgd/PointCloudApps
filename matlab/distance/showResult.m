clc
clear

radii = [4, 6];
numTracks = 50;

for radius = radii
    for index=1:3
        dsetPath = workingDir(index, radius)
        resFile = sprintf('%s/BD_%d.mat', dsetPath, numTracks);
        BD = load(resFile);
        BD = BD.BD;
        
        BDv.prcv = BD.prcv(BD.prcv~=0 & BD.prcv~=inf);
        BDv.pcwg = BD.pcwg(BD.pcwg~=0 & BD.pcwg~=inf);
        BDv.spin = BD.spin(BD.spin~=0 & BD.spin~=inf);
        BDv.fpfh = BD.fpfh(BD.fpfh~=0 & BD.fpfh~=inf);
        BDv.shot = BD.shot(BD.shot~=0 & BD.shot~=inf);
        BDv.tris = BD.tris(BD.tris~=0 & BD.tris~=inf);
        
        BDv.prcv = sqrt(BDv.prcv);
        BDv.pcwg = sqrt(BDv.pcwg);
        BDv.spin = sqrt(BDv.spin);
        BDv.fpfh = sqrt(BDv.fpfh);
        BDv.shot = sqrt(BDv.shot);
        BDv.tris = sqrt(BDv.tris);
        
        result = real([mean(BDv.prcv) mean(BDv.pcwg) mean(BDv.spin) ...
                        mean(BDv.fpfh) mean(BDv.shot) mean(BDv.tris)])
    end
end
