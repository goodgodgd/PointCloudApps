function dstpath = workingDir(dsetIndex, radius)

rootfolder = 'E:\PaperData\rgbd-scene-dataset';

if dsetIndex == 1
    dstpath = sprintf('%s/corbs-cabinet2/DescriptorR%d', rootfolder, radius);
elseif dsetIndex == 2
    dstpath = sprintf('%s/corbs-desk2/DescriptorR%d', rootfolder, radius);
elseif dsetIndex == 3
    dstpath = sprintf('%s/corbs-human2/DescriptorR%d', rootfolder, radius);
else
    error('dataset index out of bound')
end

if exist(dstpath, 'dir')==0
    error('path does not exist: %s', dstpath)
end

end