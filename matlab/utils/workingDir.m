function dstpath = workingDir(dsetIndex)

rootfolder = 'E:\PaperData\rgbd-scene-dataset';
subfolder = 'Descriptor1';

if dsetIndex == 1
    dstpath = sprintf('%s/icl-nuim-livingroom1/%s', rootfolder, subfolder);
elseif dsetIndex == 2
    dstpath = sprintf('%s/icl-nuim-livingroom1-noisy/%s', rootfolder, subfolder);
elseif dsetIndex == 3
    dstpath = sprintf('%s/icl-nuim-office1/%s', rootfolder, subfolder);
elseif dsetIndex == 4
    dstpath = sprintf('%s/icl-nuim-office1-noisy/%s', rootfolder, subfolder);
elseif dsetIndex == 5
    dstpath = sprintf('%s/tum_freiburg1_desk/%s', rootfolder, subfolder);
elseif dsetIndex == 6
    dstpath = sprintf('%s/tum_freiburg1_room/%s', rootfolder, subfolder);
elseif dsetIndex == 7
    dstpath = sprintf('%s/tum_freiburg2_desk/%s', rootfolder, subfolder);
elseif dsetIndex == 8
    dstpath = sprintf('%s/tum_freiburg3_long/%s', rootfolder, subfolder);
else
    error('dataset index out of bound')
end

if exist(dstpath, 'dir')==0
    error('path does not exist: %s', dstpath)
end

end